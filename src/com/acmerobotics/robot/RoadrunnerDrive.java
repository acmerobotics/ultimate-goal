package com.acmerobotics.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.robomatic.hardware.CachingSensor;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.acmerobotics.robomatic.util.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//PS: rn i'm writing this with the motor encoders. Later, i'll add in the omni trackers.
@Config
public class RoadrunnerDrive extends Subsystem {

    //constants
    private static final double WHEEL_RADIUS = 2;

    private static final double TRACKER_RADIUS = DistanceUnit.INCH.fromMm(35.0 / 2.0);
    private static final double TRACKER_TICKS_PER_INCH = (500 * 4) / (2 * TRACKER_RADIUS * Math.PI);

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double HEADING_P = 0;
    public static double HEADING_I = 0;
    public static double HEADING_D = 0;
    public static PIDCoefficients COEFFICIENTS = new PIDCoefficients(P, I, D);
    public static PIDCoefficients HEADINGPID = new PIDCoefficients(HEADING_P, HEADING_I, HEADING_D);

    public static double K_V = 0;
    public static double K_A = 0;
    public static double K_STATIC = 0;
    public static double K_G = 0;
    public static double G = K_G * 368;

    public static double MAX_V = 0;
    public static double MAX_A = 0;
    public static double MAX_J = 0;

    public static double MAX_ANGLE_V = 0;
    public static double MAX_ANGLE_A = 0;
    public static double MAX_ANGLE_J = 0;

    //Roadrunner stuff
    private DriveConstraints driveConstraints = new DriveConstraints(MAX_V, MAX_A, MAX_J, MAX_ANGLE_V, MAX_ANGLE_A, MAX_ANGLE_J);
    private Trajectory trajectory;

    private HolonomicPIDVAFollower follower = new HolonomicPIDVAFollower(COEFFICIENTS, COEFFICIENTS, HEADINGPID);

    private long startTime;

    private Pose2d targetVelocity = new Pose2d(0, 0, 0);
    private Pose2d currentEstPose = new Pose2d(0, 0, 0);

    //Dashboard
    private FtcDashboard dashboard;

    //hardware devices
    public DcMotorEx[] motors = new DcMotorEx[4];
    public DcMotorEx omniTracker;

    // motors and motor encoder variables
    private static double MAX_VEL = 30;
    private static double MAX_O = 30;
    public static double SLOW_V = MAX_V/2;
    public static double SLOW_O = MAX_O/2;

    private double ticksPerRev = 560.0;

    // imu variables
    //Orientation lastAngle = new Orientation();
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

    private double wheelOmega = 0;

    private boolean inTeleOp = false;

    // event triggers

    private enum AutoMode{
        IDLE,
        FOLLOWING_PATH,
        OPEN_LOOP,
        HOLD_POSITION
    }

    public double target = 0;

    private AutoMode autoMode = AutoMode.IDLE;

    private LinearOpMode opMode;


    public RoadrunnerDrive(Robot robot, LinearOpMode opMode){
        super("Drive");

        this.opMode = opMode;

        for (int i=0; i<4;i++){
            motors[i] = robot.getMotor("m" + i);
        }

        BNO055IMUImpl imu = robot.getRevHubImu(0, new Robot.Orientation(Robot.Axis.POSITIVE_X, Robot.Axis.POSITIVE_Y, Robot.Axis.POSITIVE_Z)); // creates BN0055-IMU-Impl, imu orientation is remapped
        imuSensor = new CachingSensor<>(() -> imu.getAngularOrientation().firstAngle); // gets heading
        robot.registerCachingSensor(imuSensor); // adds imu to caching sensors, will then update the heading

        dashboard = FtcDashboard.getInstance();


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

    }

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
            wheelOmega = (wheelVelocity.dot(ROTOR_DIRECTIONS[i]) * Math.sqrt(2)) / WHEEL_RADIUS;
            motors[i].setVelocity(wheelOmega, AngleUnit.RADIANS);


        }

    }

    public void followTrajectory(Trajectory trajectory){
        this.trajectory = trajectory;
        autoMode = AutoMode.FOLLOWING_PATH;
        startTime = System.currentTimeMillis();
    }

    public boolean isFollowingPath(){
        return autoMode == AutoMode.FOLLOWING_PATH; //add trajectory is complete too
    }

    public void stop(){
        targetVelocity = new Pose2d(0, 0, 0);
        autoMode = AutoMode.OPEN_LOOP;
    }

    @Override
    public void update(Canvas canvas){

        if (!inTeleOp){

            switch (autoMode){
                case IDLE:
                    // do absolutely nothing. just sit here and be a placeholder.
                    break;

                case FOLLOWING_PATH:
                    follower.followTrajectory(trajectory);
                    DriveSignal signal = follower.update(currentEstPose);

                    break;


                case OPEN_LOOP:
                    setVelocity(targetVelocity);

                    break;

                case HOLD_POSITION:
                    //stuff for holding the position, I just don't want to write it rn

                    break;

            }
        }


    }





}
