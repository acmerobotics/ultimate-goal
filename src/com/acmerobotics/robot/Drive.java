package com.acmerobotics.robot;

import android.sax.StartElementListener;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.robomatic.hardware.CachingSensor;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.acmerobotics.robomatic.robot.TelemetryData;
import com.acmerobotics.robomatic.util.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public class Drive extends Subsystem{

    //TODO: fix drive coordinate plan. Should be positve x forward and positive y to the robots left. I have it using reg coordinate plane right now

    //constants
    private static final double WHEEL_RADIUS = 2;

    private static final double TRACKER_RADIUS = DistanceUnit.INCH.fromMm(35.0 / 2.0);
    private static final double TRACKER_TICKS_PER_REV = 8192;

    //hardware devices
    public DcMotorEx[] motors = new DcMotorEx[4];
    public DcMotorEx omniTrackerX;
    public DcMotorEx omniTrackerY;

    // motors and motor encoder variables
    private static double MAX_V = 30;
    private static double MAX_O = 30;
    public static double SLOW_V = MAX_V/2;
    public static double SLOW_O = MAX_O/2;

    private double ticksPerRev = 560.0;

    // imu variables
    //Orientation lastAngle = new Orientation();
    private CachingSensor imuSensor;
    private double lastAngle = 0;
    private double globalAngle;

    // vector/pos variables
    private static Vector2d[] WHEEL_POSITIONS = {
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

    private double wheelOmega = 0;

    // correction variables

    public double error0;
    public double error1;
    public double error2;
    public double error3;


    private boolean inTeleOp;

    private PIDController pidController;
    private PIDController turnPidController;
    private PIDController strafePidController;
    private PIDController correctionPidController;

    public static double P = 0.0003;
    public static double I = 0;
    public static double D = 0.00007;

    public static double tP = 0.035;
    public static double tI = 0;
    public static double tD = 0.003;

    public static double sP = 0.00045;
    public static double sI = 0;
    public static double sD = 0.00005;

    public static double cP = 0.05; // 0.3
    public static double cI = 0;
    public static double cD = 0;

    private enum AutoMode{
        UNKNOWN,
        Y,
        STRAFE,
        TURN
    }

    private double Ytarget = 0;
    private double Xtarget = 0;
    private double turnTarget = 0;

    public double correction0 = 0;
    public double correction1 = 0;
    public double correction2 = 0;
    public double correction3 = 0;

    public double target = 0;

    // should be changed if needed (in ticks)
    public static double YErrorTolerance = 1000;
    public static double XErrorTolerance = 1000;
    public static double headingErrorTolerance = 5;



    private AutoMode autoMode = AutoMode.UNKNOWN;

    private LinearOpMode opMode;


    public Drive(Robot robot, LinearOpMode opMode){
        super("Drive");

        this.opMode = opMode;

        inTeleOp = isInTeleOp(); // anything in telOp folder will be considered a teleOp opMOde

        BNO055IMUImpl imu = robot.getRevHubImu(0, new Robot.Orientation(Robot.Axis.POSITIVE_X, Robot.Axis.POSITIVE_Y, Robot.Axis.POSITIVE_Z)); // creates BN0055-IMU-Impl, imu orientation is remapped
        imuSensor = new CachingSensor<>(() -> imu.getAngularOrientation().firstAngle); // gets heading
        robot.registerCachingSensor(imuSensor); // adds imu to caching sensors, will then update the heading

        pidController = new PIDController(P, I, D);
        turnPidController = new PIDController(tP, tI, tD);
        strafePidController = new PIDController(sP, sI, sD);
        correctionPidController = new PIDController(cP, cI, cD);

        for (int i=0; i<4;i++){
            motors[i] = robot.getMotor("m" + i);
        }

        omniTrackerX = robot.getMotor("omniX");
        omniTrackerY = robot.getMotor("intakeMotor");

        if(!inTeleOp){
           omniTrackerX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           omniTrackerY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motors[0].setDirection(DcMotorEx.Direction.REVERSE);
            motors[1].setDirection(DcMotorEx.Direction.REVERSE);
            motors[2].setDirection(DcMotorEx.Direction.FORWARD);
            motors[3].setDirection(DcMotorEx.Direction.FORWARD);

            for (int i = 0; i < 4; i++){
                motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
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

        //Orientation heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }


    @Override
    public void update(Canvas overlay){
        telemetryData.addData("currentPos0", motors[0].getCurrentPosition());
        telemetryData.addData("currentPos1", motors[1].getCurrentPosition());
        telemetryData.addData("currentPos2", -motors[2].getCurrentPosition());
        telemetryData.addData("currentPos3", -motors[3].getCurrentPosition());
        telemetryData.addData("X Position", omniTrackerX.getCurrentPosition());
        telemetryData.addData("Y Position", omniTrackerY.getCurrentPosition());
        telemetryData.addData("X position inches", strafeCurrentPositionInches());
        telemetryData.addData("Y position inches", omniTicksPerInch(omniTrackerY.getCurrentPosition()));


        telemetryData.addData("target", target);

        telemetryData.addData("m0 power ", motors[0].getPower());
        telemetryData.addData("m1 power ", motors[1].getPower());
        telemetryData.addData("m2 power ", motors[2].getPower());
        telemetryData.addData("m3 power ", motors[3].getPower());

        telemetryData.addData("corr 0", correction0);
        telemetryData.addData("corr 1", correction1);

        telemetryData.addData("heading (degrees)", getAngle());
        telemetryData.addData("inTeleOp ", inTeleOp);

        telemetryData.addData("autoMode ", autoMode);

        if (!inTeleOp){

            // adjust error for a motor power
            pidController.setOutputBounds(-0.9, 0.9);
            turnPidController.setOutputBounds(-1, 1);
            strafePidController.setOutputBounds(-0.8, 0.8);
            correctionPidController.setOutputBounds(-0.5, 0.5);


            switch (autoMode){
                case UNKNOWN:
                    motors[0].setPower(0);
                    motors[1].setPower(0);
                    motors[2].setPower(0);
                    motors[3].setPower(0);

                    break;

                case Y:

                    error0 = target - omniTrackerY.getCurrentPosition();
                    error1 = getAngle(); // used to correct heading

                    correction0 = pidController.update(error0);
                    correction1 = correctionPidController.update(error1); // used to correct heading

                    motors[0].setPower(correction0 + correction1);
                    motors[1].setPower(correction0 + correction1);
                    motors[2].setPower(correction0);
                    motors[3].setPower(correction0);

                    break;

                case STRAFE:

                    error0 = target - omniTrackerX.getCurrentPosition();
                    error1 = getAngle(); // used to correct heading

                    correction0 = strafePidController.update(error0);
                    correction1 = correctionPidController.update(error1); // used to correct heading

                    motors[0].setPower(correction0);
                    motors[1].setPower(-correction0 - -correction1);
                    motors[2].setPower(correction0 - correction1);
                    motors[3].setPower(-correction0);

                    break;

                case TURN:

                    error0 = target - getAngle();

                    correction0 = turnPidController.update(error0);

                    motors[0].setPower(-correction0);
                    motors[1].setPower(-correction0);
                    motors[2].setPower(correction0);
                    motors[3].setPower(correction0);

                    break;

            }
        }
    }


    //////////////////////////////////  TeleOP  ////////////////////////////////////////////

    public void setPower(Pose2d target) {
        double v = target.vec().norm() * MAX_V;
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


    public void setVelocity(Pose2d v) {
        for (int i = 0; i < 4; i++) {
            Vector2d wheelVelocity = new Vector2d(v.getX() - v.getHeading() * WHEEL_POSITIONS[i].getY(),
                    v.getY() + v.getHeading() * WHEEL_POSITIONS[i].getX());
            wheelOmega = (wheelVelocity.dot(ROTOR_DIRECTIONS[i]) * Math.sqrt(2)) / WHEEL_RADIUS;
            motors[i].setVelocity(wheelOmega, AngleUnit.RADIANS);


        }

    }


    ///////////////////////////////////////// Auto ////////////////////////////////////////////

    // linear movements

    public void moveForward(double inches){
        Ytarget = omniEncodersInchesToTicks(inches);

        prepareMotors();

        target = Ytarget; // motors[0].getCurrentPosition() + Ytarget;
        error0 = YErrorTolerance;

        autoMode = AutoMode.Y;
    }

    public void moveBack(double inches){
        Ytarget = omniEncodersInchesToTicks(-inches);

        prepareMotors();

        target = Ytarget; //motors[0].getCurrentPosition() + Ytarget;
        error0 = YErrorTolerance;

        autoMode = AutoMode.Y;
    }

    public void strafeRight(double inches){
        //Xtarget = motorEncodersInchesToTicks(inches); // omni encoder ticks to inches

        Xtarget = omniEncodersInchesToTicks(inches);

        prepareMotors();

        target = Xtarget; // motors[0].getCurrentPosition() + Xtarget;
        error0 = XErrorTolerance;

        autoMode = AutoMode.STRAFE;
    }

    public void strafeLeft(double inches){
        //Xtarget = motorEncodersInchesToTicks(-inches); // omni encoder ticks to inches

        Xtarget = omniEncodersInchesToTicks(-inches);

        prepareMotors();

        target = Xtarget; // motors[0].getCurrentPosition() + Xtarget;
        error0 = XErrorTolerance;

        autoMode = AutoMode.STRAFE;
    }


    public boolean atYPosition(){

        if (Math.abs(error0) < YErrorTolerance) {
            error0 = YErrorTolerance;
            return true;
        } else {
            return false;
            }
    }


    public boolean atStrafePosition(){
        if (Math.abs(error0) < XErrorTolerance){
            error0 = XErrorTolerance;
            return true;
        }
        else{
            return false;
        }
    }

    public double strafeCurrentPositionInches(){
        return omniTicksPerInch(omniTrackerX.getCurrentPosition());
    }


    // turning

    public void turnLeft(double degrees){
        turnTarget = degrees;

        prepareMotors();

        target = getAngle() + turnTarget;
        error0 = headingErrorTolerance;

        autoMode = AutoMode.TURN;
    }

    public void turnRight(double degrees){
        turnTarget = -degrees;

        prepareMotors();

        target = getAngle() + turnTarget;
        error0 = headingErrorTolerance;

        autoMode = AutoMode.TURN;
    }

    public void turnTo(double angle){
        prepareMotors();

        target = angle;

        autoMode = AutoMode.TURN;
    }


    public boolean atTurningPosition() {
        if (Math.abs(error0) < headingErrorTolerance) {
            error0 = headingErrorTolerance;
            return true;
        } else {
            return false;
        }
    }

    public void resetAngle(){

        //lastAngle = Math.toDegrees((float) imuSensor.getValue());

        globalAngle = 0;

    }

    public double getAngle(){

        double angle;

        if (imuSensor.getValue() != null){
            angle = Math.toDegrees((Double.parseDouble(imuSensor.getValue().toString()))); // radians to degrees
        }
        else {
            angle = lastAngle;
        }

        //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angle - lastAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngle = angle;

        return globalAngle;

    }


    private int omniEncodersInchesToTicks(double inches) {
        double circumference = 2 * Math.PI * TRACKER_RADIUS;
        return (int) Math.round(inches * TRACKER_TICKS_PER_REV / circumference);
    }

    private int motorEncodersInchesToTicks(double inches) {
        double circumference = 2 * Math.PI * WHEEL_RADIUS;
        return (int) Math.floor(inches * ticksPerRev / circumference);
    }


    // misc

    private double omniTicksPerInch(int ticks){
        double circumference = 2 * Math.PI * TRACKER_RADIUS;

        return (circumference *  ticks / TRACKER_TICKS_PER_REV);
    }

    public double ticksToInches(int ticks) {
        double revs = ticks / ticksPerRev;
        return 2 * Math.PI * WHEEL_RADIUS * revs;

    }

    public void stopMotors(){
        autoMode = AutoMode.UNKNOWN;
        motors[0].setPower(0);
        motors[1].setPower(0);
        motors[2].setPower(0);
        motors[3].setPower(0);

    }

    private double convertToDegrees(double radians){
        return (180 * radians) / 3.14;
    }

    public void resetEncoderOmni(){
        omniTrackerX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        omniTrackerY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void resetEncoders(){
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void stopAndResetMotors(){
        stopMotors();
        resetEncoders();
        resetEncoderOmni();
    }


    private boolean isInTeleOp (){
        String opMode = String.valueOf(this.opMode);

        int hasTeleOp = opMode.indexOf("teleop");

        return hasTeleOp != -1;
    }

    private void prepareMotors(){
        for (int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            omniTrackerX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            omniTrackerY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

}
