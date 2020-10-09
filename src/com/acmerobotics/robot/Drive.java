package com.acmerobotics.robot;

import android.sax.StartElementListener;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.opmodes.teleop.DriveTest;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    //constants
    private static final double WHEEL_RADIUS = 2;

    private static final double TRACKER_RADIUS = DistanceUnit.INCH.fromMm(35.0 / 2.0);
    private static final double TRACKER_TICKS_PER_INCH = (500 * 4) / (2 * TRACKER_RADIUS * Math.PI);

    //hardware devices
    public DcMotorEx[] motors = new DcMotorEx[4];
    public DcMotorEx omniTracker;

    // motors and motor encoder variables
    private static double MAX_V = 30;
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
    public double Pcoefficient = 0.1; // 0.2
    public static double PcoefficientTurn = 0.04;

    public double error0;
    public double error1;
    public double error2;
    public double error3;

    public double newPower;

    private boolean inTeleOp = false;

    private PIDController pidController;

    public static double P = 0.005;
    public static double I = 0;
    public static double D = 0;

    // event triggers

    private enum AutoMode{
        UNKNOWN,
        Y,
        STRAFE,
        TURN
    }

    public double Ytarget = 0;
    private double Xtarget = 0;
    private double turnTarget = 0;
    public double correction;

    private double robotPower;

    private int targetPosition = 0;
    private int targetMotorPos;

    public double correction0 = 0;
    public double correction1 = 0;
    public double correction2 = 0;
    public double correction3 = 0;

    public double target = 0;

    public boolean canSetYTarget = true;
    public boolean canSetXTarget = true;
    public boolean canSetTurnTarget = true;

    // should be changed if needed (in ticks)
    private double YErrorTolerance = 3;
    private double XErrorTolerance = 5;
    private double headingErrorTolerance = 5;



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

        for (int i=0; i<4;i++){
            motors[i] = robot.getMotor("m" + i);
        }



        if(!inTeleOp){
            //omniTracker = robot.getMotor("intakeMotor");
           // omniTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motors[0].setDirection(DcMotorEx.Direction.REVERSE);
            motors[1].setDirection(DcMotorEx.Direction.REVERSE);
            motors[2].setDirection(DcMotorEx.Direction.FORWARD);
            motors[3].setDirection(DcMotorEx.Direction.FORWARD);

            for (int i = 0; i < 4; i++){
                motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motors[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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

        telemetryData.addData("target", target);
        telemetryData.addData("m0 power ", motors[0].getPower());

        telemetryData.addData("heading (radians)", imuSensor.getValue());
        telemetryData.addData("heading (degrees)", getAngle());
        telemetryData.addData("heading imu (degrees)", Math.toDegrees((Double.parseDouble(imuSensor.getValue().toString()))) );
        telemetryData.addData("inTeleOp ", inTeleOp);

        telemetryData.addData("autoMode ", autoMode);

        if (!inTeleOp){

            // adjust error for a motor power
            //error = error /  12; // might move this statement to a method
            pidController.setOutputBounds(-1, 1);


            switch (autoMode){
                case UNKNOWN:
                    break;

                case Y:

                    error0 = target - motors[0].getCurrentPosition();
                    error1 = target - motors[1].getCurrentPosition();
                    error2 = target - -motors[2].getCurrentPosition();
                    error3 = target - -motors[3].getCurrentPosition();

                    correction0 = pidController.update(error0);
                    correction1 = pidController.update(error1);
                    correction2 = pidController.update(error2);
                    correction3 = pidController.update(error3);

                    motors[0].setPower(correction0);
                    motors[1].setPower(correction1);
                    motors[2].setPower(correction2);
                    motors[3].setPower(correction3);


                    break;


                case STRAFE:


                    // TODO add strafe alignment correction
                    // TODO add functionality with omni wheel
                    // the target and error are based on motors 0 and 2 but the values and the same
                    // for motors 1 and 3 but opposite sign

                    error0 = target - motors[0].getCurrentPosition();  // strafeCurrentPosition()
                    error1 = -target - motors[1].getCurrentPosition();
                    error2 = target - -motors[2].getCurrentPosition();
                    error3 = -target - -motors[3].getCurrentPosition();

                    correction0 = pidController.update(error0);
                    correction1 = pidController.update(error1);
                    correction2 = pidController.update(error2);
                    correction3 = pidController.update(error3);

                    motors[0].setPower(correction0);
                    motors[1].setPower(correction1);
                    motors[2].setPower(correction2);
                    motors[3].setPower(correction3);

                    break;

                case TURN:

                    error0 = target - getAngle();

                    correction0 = pidController.update(error0);
                    correction1 = pidController.update(error0);
                    correction2 = pidController.update(error0);
                    correction3 = pidController.update(error0);

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
        Ytarget = motorEncodersInchesToTicks(inches);

        if (canSetYTarget) {
            target = motors[0].getCurrentPosition() + Ytarget;
            canSetYTarget = false;
        }

        autoMode = AutoMode.Y;
    }

    public void moveBack(double inches){
        Ytarget = motorEncodersInchesToTicks(-inches);

        if (canSetYTarget) {
            target = motors[0].getCurrentPosition() + Ytarget;
            canSetYTarget = false;
        }

        autoMode = AutoMode.Y;
    }

    public void strafeRight(double inches){
        Xtarget = motorEncodersInchesToTicks(inches); // strafeCurrentPosition() // omni encoder ticks to inches

        if (canSetXTarget) {
            target = motors[0].getCurrentPosition() + Xtarget;
            canSetXTarget = false;
        }
        autoMode = AutoMode.STRAFE;
    }

    public void strafeLeft(double inches){
        Xtarget = motorEncodersInchesToTicks(-inches); // strafeCurrentPosition() // omni encoder ticks to inches

        if (canSetXTarget) {
            target = motors[0].getCurrentPosition() + Xtarget;
            canSetXTarget = false;
        }

        autoMode = AutoMode.STRAFE;
    }


    public boolean atYPosition(){
        if (Math.abs(error0) < YErrorTolerance){
            canSetYTarget = true;
            return true;
        }
        else{
            return false;
        }
    }


    public boolean atStrafePosition(){
        if (Math.abs(error0) < XErrorTolerance){
            canSetXTarget = true;
            return true;
        }
        else{
            return false;
        }
    }

    public double strafeCurrentPosition(){
        return omniTicksPerInch(omniTracker.getCurrentPosition());
    }


    // turning

    public void turnLeft(double degrees){
        turnTarget = degrees;

        if (canSetTurnTarget) {
            target = getAngle() + turnTarget;
            canSetTurnTarget = false;
        }

        autoMode = AutoMode.TURN;
    }

    public void turnRight(double degrees){
        turnTarget = -degrees;

        if (canSetTurnTarget) {
            target = getAngle() + turnTarget;
            canSetTurnTarget = false;
        }

        autoMode = AutoMode.TURN;
    }


    public boolean atTurningPosition() {
        if (Math.abs(error0) < headingErrorTolerance) {
            canSetTurnTarget = true;
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

        double angle = Math.toDegrees((Double.parseDouble(imuSensor.getValue().toString()))); // radians to degrees
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


    private int omniEncodersInchesToTicks(int inches) {
        double circumference = 2 * Math.PI * TRACKER_RADIUS;
        return (int) Math.round(inches * (500 * 4) / circumference);
    }

    private int motorEncodersInchesToTicks(double inches) {
        double circumference = 2 * Math.PI * WHEEL_RADIUS;
        return (int) Math.floor(inches * ticksPerRev / circumference);
    }


    ///////////////////////////////////////Angle Corrector//////////////////////////////////////////

    public void setZero(){
        resetAngle();
    }


    private void setError(){
        error0 = getAngle();
    }


    private double Pcontroller(){
        double output = Pcoefficient * error0;
        return output;
    }


    private double PcontrollerTurn(){
        return PcoefficientTurn * error0;
    }


    public void correctingPower(double defaultPower, int motorNum, String direction){
        setError();

        double correctionPower = Pcontroller();
        double turnCorrectionPower = PcontrollerTurn();

        double changeSign = Math.copySign(1, defaultPower); // 1 or -1

        if (defaultPower != 0) {

            if (direction.equals("right")) {
                newPower = defaultPower - (correctionPower * changeSign);
            }

            else if (direction.equals("left")) {
                newPower = defaultPower + (correctionPower * changeSign); // added instead of subtracted bc opposite adjustment to error is needed from right strafe
            }
        }

        else {
            if (motorNum == 0 || motorNum == 1) {
                newPower = defaultPower - turnCorrectionPower;
            }

            else { // motors 2 and 3
                newPower = defaultPower + turnCorrectionPower;
            }
        }

        motors[motorNum].setPower(newPower);
    }


    // misc

    private double omniTicksPerInch(int ticks){
        double D = 1.4;
        int ticksPerRev = 2000;
        double circumference = D * 3.14;

        return (circumference *  ticks / ticksPerRev);
    }

    public double ticksToInches(int ticks) {
        double revs = ticks / ticksPerRev;
        return 2 * Math.PI * WHEEL_RADIUS * revs;

    }

    public void stopMotors(){
        motors[0].setPower(0);
        motors[1].setPower(0);
        motors[2].setPower(0);
        motors[3].setPower(0);

    }

    private double convertToDegrees(double radians){
        return (180 * radians) / 3.14;
    }

    public void resetEncoderOmni(){
        omniTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

}
