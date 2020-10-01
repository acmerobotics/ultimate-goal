package com.acmerobotics.robot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.robomatic.hardware.CachingSensor;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TestSubsystem extends Subsystem {

    public DcMotor motor;
    public CachingSensor imuSensor;
//    private CachingSensor colorSensor;
//    private Servo servo;
//    private DigitalChannel hallEffect;

    public TestSubsystem(Robot robot, HardwareMap hardwareMap){ // be default only robot is a parameter but I had to
                                                                // include hardwareMap for the color sensor
        super("TestSubsystem");

        motor = robot.getMotor("motor1");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //servo = robot.getServo("servo");

//        ColorSensor colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor");
//        colorSensor = new CachingSensor<>(() -> colorSensor1.blue());
//        robot.registerCachingSensor(colorSensor);

//        hallEffect = robot.getDigitalChannel("hallEffect");

        BNO055IMUImpl imu = robot.getRevHubImu(0, new Robot.Orientation(
                Robot.Axis.POSITIVE_X,
                Robot.Axis.POSITIVE_Y,
                Robot.Axis.POSITIVE_Z));

        imuSensor = new CachingSensor<>(() -> imu.getAngularOrientation().firstAngle);
        robot.registerCachingSensor(imuSensor);


    }

    @Override
    public void update(Canvas overlay){
        telemetryData.addData("power", motor.getPower());
        telemetryData.addData("heading", Math.toDegrees((float) imuSensor.getValue()));
//        telemetryData.addData("color sensor", colorSensor.getValue());
//        telemetryData.addData("hallEffect", !hallEffect.getState());
    }

    public void setPower(double power){
        motor.setPower(power);
    }

//    public void servoPositionOne() {
//        servo.setPosition(.2);
//    }
//
//    public void servoPositionTwo() {
//        servo.setPosition(.8);
//    }

}


