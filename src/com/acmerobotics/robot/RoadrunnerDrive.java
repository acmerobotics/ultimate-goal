package com.acmerobotics.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.acmerobotics.robomatic.hardware.CachingSensor;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.acmerobotics.util.DriveContants;
import com.acmerobotics.util.TwoWheelTrackingLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static com.acmerobotics.util.DriveContants.BASE_CONSTRAINTS;
import static com.acmerobotics.util.DriveContants.TRACK_WIDTH;

@Config
public class RoadrunnerDrive extends Subsystem {

    //constants
    private static final double TRACKER_RADIUS = DistanceUnit.INCH.fromMm(35.0 / 2.0);
    private static final double TRACKER_TICKS_PER_INCH = (500 * 4) / (2 * TRACKER_RADIUS * Math.PI);

    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    //Roadrunner stuff
    private FtcDashboard dashboard;
    private NanoClock clock;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private DriveConstraints constraints;
    private TrajectoryFollower follower;

    private long startTime;

    private MecanumDrive mecanumDrive;

    private Pose2d lastPoseOnTurn;

    //hardware devices
    public DcMotorEx[] motors = new DcMotorEx[4];
    public DcMotorEx omniTracker;

    // motors and motor encoder variables
    private static double MAX_VEL = 30;
    private static double MAX_O = 30;
    public static double SLOW_V = MAX_VEL/2;
    public static double SLOW_O = MAX_O/2;

    private double ticksPerRev = 560.0;

    // imu variables
    //Orientation lastAngle = new Orientation();
    private BNO055IMU imu;
    public CachingSensor imuSensor;
    private double lastAngle = 0;
    private double globalAngle;

    // servo positions
    private double grabPosition = 0.75;
    private double releasePosition = 0.20;

    // vector/pos variables
    private static Vector2d[] WHEEL_POSITIONS = { //remember we'll have to remeasure these for the new drive base
            new Vector2d(6, 7.5),
            new Vector2d(-6, 7.5),
            new Vector2d(-6, -7.5),
            new Vector2d(6, -7.5)
    };

    private static Vector2d[] ROTOR_DIRECTIONS = {
            new Vector2d(1, 1),
            new Vector2d(-1, 1),
            new Vector2d(-1, -1),
            new Vector2d(1, -1)
    };

    private Pose2d targetVelocity = new Pose2d(0, 0, 0);
    private Pose2d currentEstimatedPose = new Pose2d(0, 0, 0);
    private Pose2d currentEstimatedPoseTrackers = new Pose2d(0, 0, 0);


    private double wheelOmega = 0;

    private boolean inTeleOp = false;

    // event triggers
    private enum AutoMode{
        IDLE,
        TURN,
        FOLLLOW_TRAJECTORY,
    }

    private AutoMode autoMode = AutoMode.IDLE;

    private LinearOpMode opMode;


    public RoadrunnerDrive(Robot robot, LinearOpMode opMode){
        super("Roadrunnner_Drive");

        this.opMode = opMode;

        dashboard = FtcDashboard.getInstance();
        clock = NanoClock.system();

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID);

        for (int i=0; i<4;i++){
            motors[i] = robot.getMotor("m" + i);
        }

        imu = robot.getRevHubImu(0, new Robot.Orientation(Robot.Axis.POSITIVE_X, Robot.Axis.POSITIVE_Y, Robot.Axis.POSITIVE_Z)); // creates BN0055-IMU-Impl, imu orientation is remapped
        imuSensor = new CachingSensor<>(() -> imu.getAngularOrientation().firstAngle); // gets heading
        robot.registerCachingSensor(imuSensor); // adds imu to caching sensors, will then update the heading



        if(!inTeleOp){
            //TODO: add in omni trackers

            motors[0].setDirection(DcMotorEx.Direction.REVERSE);
            motors[1].setDirection(DcMotorEx.Direction.REVERSE);
            motors[2].setDirection(DcMotorEx.Direction.FORWARD);
            motors[3].setDirection(DcMotorEx.Direction.FORWARD);

            for (int i = 0; i < 4; i++){
                motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motors[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            }



        } else {

            //imuSensor.setEnabled(false); //disable the imu if it is not needed (you might have to
            // remove te sensor from the
            // teleOp telemetry)

            motors[0].setDirection(DcMotorEx.Direction.FORWARD);
            motors[1].setDirection(DcMotorEx.Direction.REVERSE);
            motors[2].setDirection(DcMotorEx.Direction.FORWARD);
            motors[3].setDirection(DcMotorEx.Direction.REVERSE);

            for (int i = 0; i < 4; i++){
                motors[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            }

        }

        mecanumDrive.setLocalizer(new TwoWheelTrackingLocalizer(robot));
    }

    //TeleOp Drive
    public void setPower(Pose2d target) { //set velocity (for joystick transform and such)
        double v = target.vec().norm() * MAX_VEL;
        double theta = Math.atan2(target.getX(), target.getY());
        double omega = target.getHeading() * MAX_O;

        targetVelocity = new Pose2d(v * Math.cos(theta), v * Math.sin(theta), omega);

        setVelocity(targetVelocity);

    }

    public void setSlowPower(Pose2d target) {
        double v = target.vec().norm() * SLOW_V;
        double theta = Math.atan2(target.getX(), target.getY());
        double omega = target.getHeading() * SLOW_O;

        targetVelocity = new Pose2d(v * Math.cos(theta), v * Math.sin(theta), omega);

        setVelocity(targetVelocity);
    }

    public void setVelocity(Pose2d v) { //internal set velocity
        for (int i = 0; i < 4; i++) {
            Vector2d wheelVelocity = new Vector2d(v.getX() - v.getHeading() * WHEEL_POSITIONS[i].getY(),
                    v.getY() + v.getHeading() * WHEEL_POSITIONS[i].getX());
            wheelOmega = (wheelVelocity.dot(ROTOR_DIRECTIONS[i]) * Math.sqrt(2)) / DriveContants.WHEEL_RADIUS;
            motors[i].setVelocity(wheelOmega, AngleUnit.RADIANS);


        }

    }

    //Roadrunner/autonomous code

    //Trajectory Builders
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose){
        return new TrajectoryBuilder(startPose, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed){
        return new TrajectoryBuilder(startPose, reversed, constraints);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, constraints);
    }

    public void turnAsync(double angle){
        double heading = mecanumDrive.getPoseEstimate().getHeading();

        lastPoseOnTurn = mecanumDrive.getPoseEstimate();

        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(heading, 0, 0, 0),
                new MotionState(heading + angle, 0, 0, 0),
                constraints.maxAngVel,
                constraints.maxAngAccel,
                constraints.maxAngJerk
        );

        turnStart = clock.seconds();
        autoMode = AutoMode.TURN;
    }

    public void followTrajectoryAsync(Trajectory trajectory){
        follower.followTrajectory(trajectory);
        autoMode = AutoMode.FOLLLOW_TRAJECTORY;
    }

    public void followTrajectory(Trajectory trajectory){
        followTrajectoryAsync(trajectory);
    }

    public Pose2d getLastError() {
        switch (autoMode) {
            case FOLLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

    @Override
    public void update(Canvas overlay){
        mecanumDrive.updatePoseEstimate();

        Pose2d currentPose = mecanumDrive.getPoseEstimate();
        Pose2d lastError = getLastError();

        TelemetryPacket packet = new TelemetryPacket();
        overlay = packet.fieldOverlay();

        //stuff to add to the packet/telemetry

        if (!inTeleOp){

            switch (autoMode){
                case IDLE:
                    // do absolutely nothing. just sit here and be a placeholder.
                    break;

                case TURN:
                    double t = clock.seconds() - turnStart;

                    MotionState targetState = turnProfile.get(t);

                    turnController.setTargetPosition(targetState.getX());

                    double correction = turnController.update(currentPose.getHeading());

                    double targetOmega = targetState.getV();
                    double targetAlpha = targetState.getA();
                    mecanumDrive.setDriveSignal(new DriveSignal(new Pose2d(
                            0, 0, targetOmega + correction
                    ), new Pose2d(
                            0, 0, targetAlpha
                    )));

                    Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());

                    if (t >= turnProfile.duration()) {
                        autoMode = AutoMode.IDLE;
                        mecanumDrive.setDriveSignal(new DriveSignal());
                    }

                    break;


                case FOLLLOW_TRAJECTORY:
                    mecanumDrive.setDriveSignal(follower.update(currentPose));

                    Trajectory trajectory = follower.getTrajectory();

                    if (!follower.isFollowing()) {
                        autoMode = AutoMode.IDLE;
                        mecanumDrive.setDriveSignal(new DriveSignal());
                    }


                    break;

            }
        }

        dashboard.sendTelemetryPacket(packet);
    }

    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public void setCurrentEstimatedPose(Pose2d pose){
        lastAngle = imu.getAngularOrientation().firstAngle;
        currentEstimatedPoseTrackers = pose;
        currentEstimatedPose = pose;
    }

}
