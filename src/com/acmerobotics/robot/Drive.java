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
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private PIDController strafeHeadingCorrectionPidController;
    private PIDController correctionPidController;
    private PIDController strafeCorrectionPIDController;

    public static double P = 0.0001;
    public static double I = 0;
    public static double D = 0.000025;

    public static double tP = 0.035;
    public static double tI = 0;
    public static double tD = 0.003;

    public static double sP = 0.00045;
    public static double sI = 0;
    public static double sD = 0.00005;

    public static double cP = 0.1;
    public static double cI = 0;
    public static double cD = 0;

    public static double shcP = 0.1; // 0.5 and 0.1 work on y axis but not as well on strafes, there is to many occilations
    public static double shcI = 0;
    public static double shcD = 0;

    public static double scP = 0.00009;
    public static double scI = 0;
    public static double scD = 0.00005;

    private double currPosition = 0;
    private double lastPosition;
    private double lastPosition2;

    private ElapsedTime time = new ElapsedTime();

    private double currTime = 0;
    private double lastTime;

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

    private double der = 0;
    private double der2 = 0;


    private double dy = 0;
    private double dx = 0;

    public AutoMode autoMode = AutoMode.UNKNOWN;

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
        strafeHeadingCorrectionPidController = new PIDController(shcP, shcI, shcD);
        strafeCorrectionPIDController = new PIDController(scP, scI, scD);

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
                motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
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
        telemetryData.addData("y target", omniTicksPerInch((int)target));


        //telemetryData.addData("at Y position", atYPosition2());
//        telemetryData.addData("at X position", atStrafePosition2());
        //telemetryData.addData("at Turning position", atTurningPosition2());

        telemetryData.addData("der", der);
        telemetryData.addData("der2", der2);
        telemetryData.addData("dy", dy);
        telemetryData.addData("dx", dx);

        telemetryData.addData("currPosition", currPosition);
        telemetryData.addData("lastPosition", lastPosition);

        telemetryData.addData("currTime", currTime);
        telemetryData.addData("lastTime", lastTime);


        telemetryData.addData("target", target);

        telemetryData.addData("m0 power ", motors[0].getVelocity());
        telemetryData.addData("m1 power ", motors[1].getVelocity());
        telemetryData.addData("m2 power ", motors[2].getVelocity());
        telemetryData.addData("m3 power ", motors[3].getVelocity());

        telemetryData.addData("corr 0", correction0);
        telemetryData.addData("corr 1", correction1);
        telemetryData.addData("corr 2", correction2);


        telemetryData.addData("heading (degrees)", getAngle());
        telemetryData.addData("inTeleOp ", inTeleOp);

        telemetryData.addData("autoMode ", autoMode);

        if (!inTeleOp){

            // adjust error for a motor power
            pidController.setOutputBounds(-1, 1);
            turnPidController.setOutputBounds(-1, 1);
            strafePidController.setOutputBounds(-0.8, 0.8);
            strafeHeadingCorrectionPidController.setOutputBounds(-0.8, 0.8);
            correctionPidController.setOutputBounds(-0.8, 0.8);
            strafeCorrectionPIDController.setOutputBounds(-0.8, 0.8);


            switch (autoMode){
                case UNKNOWN:
                    motors[0].setPower(0);
                    motors[1].setPower(0);
                    motors[2].setPower(0);
                    motors[3].setPower(0);

                    break;

                case Y:

                    error0 = target - omniTrackerY.getCurrentPosition();
                    error1 = getAngle() - turnTarget; // used to correct heading //

                    correction0 = pidController.update(error0);
                    correction1 = correctionPidController.update(error1); // used to correct heading

                    motors[0].setPower(correction0 + correction1);
                    motors[1].setPower(correction0 + correction1);
                    motors[2].setPower(correction0);
                    motors[3].setPower(correction0);

                    break;

                case STRAFE:

                    error0 = target - omniTrackerX.getCurrentPosition();
                    error1 = getAngle() - turnTarget; // used to correct heading
                    error2 = omniTrackerY.getCurrentPosition();

                    correction0 = strafePidController.update(error0);
                    correction1 = strafeHeadingCorrectionPidController.update(error1); // used to correct heading
                    correction2 = strafeCorrectionPIDController.update(error2);

                    motors[0].setPower(correction0 - correction2);
                    motors[1].setPower(-correction0 - -correction1);
                    motors[2].setPower(correction0 - correction2 - correction1);
                    motors[3].setPower(-correction0);

                    //findDerivative(omniTrackerX.getCurrentPosition(), time.milliseconds());

                    break;

                case TURN:

                    turnTarget = target;

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

        time.reset();

        autoMode = AutoMode.Y;
    }

    public void moveBack(double inches){
        Ytarget = omniEncodersInchesToTicks(-inches);

        prepareMotors();

        target = Ytarget; //motors[0].getCurrentPosition() + Ytarget;
        error0 = YErrorTolerance;

        time.reset();

        autoMode = AutoMode.Y;
    }

    public void strafeRight(double inches){
        //Xtarget = motorEncodersInchesToTicks(inches); // omni encoder ticks to inches

        Xtarget = omniEncodersInchesToTicks(inches);

        prepareMotors();

        target = Xtarget; // motors[0].getCurrentPosition() + Xtarget;
        error0 = XErrorTolerance;

        time.reset();

        autoMode = AutoMode.STRAFE;
    }

    public void strafeLeft(double inches){
        //Xtarget = motorEncodersInchesToTicks(-inches); // omni encoder ticks to inches

        Xtarget = omniEncodersInchesToTicks(-inches);

        prepareMotors();

        target = Xtarget; // motors[0].getCurrentPosition() + Xtarget;
        error0 = XErrorTolerance;

        time.reset();

        autoMode = AutoMode.STRAFE;
    }


    public boolean atYPosition(){ // warning do not loop over this method in more than one place at a time

        // detect oscillation

        // if no oscillation then probably fell to steady state error so detect a derivative of 0
        double currPosition = omniTrackerY.getCurrentPosition();
        double currTime = time.milliseconds();

        dy = (currPosition - lastPosition);
        dx = (currTime - lastTime);

        der = dy / dx;

        lastPosition = currPosition;
        lastTime = currTime;

        if (atYTolerance() && der < 3 && der > -3){
            lastPosition = 0;
            lastTime = 0;
            return true;
        }
        else{
            return false;
        }
    }

    public boolean atYTolerance(){
        return  (Math.abs(error0) < YErrorTolerance);
    }


    public boolean atStrafePosition(){ // warning do not loop over this method in more than one place at a time
        // detect oscillation

        // if no oscillation then probably fell to steady state error so detect a derivative of 0

        double currPosition = omniTrackerX.getCurrentPosition();
        double currTime = time.milliseconds();

        dy = (currPosition - lastPosition);
        dx = (currTime - lastTime);

        boolean headingCorrect = atStrafeCorrectionHeading(dx);

        der = dy / dx;

        lastPosition = currPosition;
        lastTime = currTime;

        if (atXTolerance() && headingCorrect && der < 1 && der > -1){
            lastPosition = 0;
            lastTime = 0;
            return true;
        }
        else{
            return false;
        }
    }

    public boolean atXTolerance(){
        return (Math.abs(error0) < XErrorTolerance);
    }

    private boolean atStrafeCorrectionHeading(double dx){
        double currPosition = getAngle();

        double dy = (currPosition - lastPosition2);

        der2 = dy / dx;

        lastPosition2 = currPosition;

        if (Math.abs(currPosition) < 2 && der2 < 0.02 && der2 > -0.02){
            lastPosition2 = 0;
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
        // detect oscillation

        // if no oscillation then probably fell to steady state error so detect a derivative of 0
        double currPosition = getAngle();
        double currTime = time.milliseconds();

        dy = (currPosition - lastPosition);
        dx = (currTime - lastTime);

        der = dy / dx;

        lastPosition = currPosition;
        lastTime = currTime;

        if (atTurningTolerance() && der < 3 && der > -3){ // atTurningTolerance
            lastPosition = 0;
            lastTime = 0;
            return true;
        }
        else{
            return false;
        }
    }

    public boolean atTurningTolerance(){
        return (Math.abs(error0) < headingErrorTolerance);
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


    // drive aiming trajectory

    public void autoShoot(){
        // switch to isTeleOp = false
        switchWheelDirections(false);

        // determine trajectory to make a shot

        //test
        moveForward(48);
        //test

        // move drive  and shoot rings

        // when shot, switch to isTeleOp = true

        // checked when trajectory finished then restore teleOp motor directions
    }

    public boolean finishedTrajectory(Telemetry telemetry){
        if (atYPosition()) {
            switchWheelDirections(true);

            telemetry.addLine("ready for driving");

            return true;
        }
        else{
            return false;
        }
    }

    public void switchWheelDirections(boolean inTeleOp){
        this.inTeleOp = inTeleOp;
        if (inTeleOp){
            motors[0].setDirection(DcMotorEx.Direction.FORWARD);
            motors[1].setDirection(DcMotorEx.Direction.REVERSE);
            motors[2].setDirection(DcMotorEx.Direction.FORWARD);
            motors[3].setDirection(DcMotorEx.Direction.REVERSE);

            for (int i = 0; i < 4; i++){
                motors[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            }
        }

        else{
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
        }
    }

    // misc


    private void findDerivative(double currPosition, double currTime){
        this.currPosition = currPosition;
        this.currTime = currTime;

        dy = (currPosition - lastPosition);
        dx = (currTime - lastTime);

        // if no oscillation then probably fell to steady state error so detect a derivative of 0
        der = dy / dx;

        lastPosition = currPosition;
        lastTime = currTime;
    }

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

    private void stopOscillation(){
        int passes = 0; // will need to be a var outside this scope because is will get looped over
        boolean over = false;

        // if over shot and under shot, is oscillating
        if (!over && currPosition > target){
            passes += 1;
            over = true;
        }

        if (over && currPosition < target){
            passes += 1;
            over = false;
        }

        // exponentially decay the effort of the pid to get a flattened curve


    }
}
