package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.config.ConfigurationLoader;
import com.acmerobotics.robot.TestConfig;
import com.acmerobotics.robot.TestRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TestRoboOpMode")
public class TestOpMode extends LinearOpMode {

    @Override
    public void runOpMode(){

        TestRobot robot = new TestRobot(this);
        TestConfig config = (TestConfig) new ConfigurationLoader(hardwareMap.appContext).getConfig();

        robot.addTelemetry("color", config.color);
        robot.addTelemetry("delay", config.delay);
        robot.addTelemetry("latched", config.latched);

        telemetry.addData("config class", config);
        telemetry.addData("color", config.color);
        telemetry.addData("delay", config.delay);
        telemetry.addData("latched", config.latched);

        telemetry.addLine("init");
        telemetry.update();

        robot.update();

        waitForStart();


        while (!isStopRequested()){

            telemetry.addLine("start");
            telemetry.update();
            robot.update();

            if (gamepad1.right_trigger > 0){
                robot.subsystem.setPower(gamepad1.right_trigger);
            }

            else{

                robot.subsystem.setPower(0);
            }

            if (gamepad1.a){
                robot.subsystem.servoPositionOne();
            }

            if (gamepad1.b){
                robot.subsystem.servoPositionTwo();
            }

            robot.update(); // hardware devices and telemeteryData will only be updated within the Robot update()

        ////////////////////////////////////////

        // below will mostly be useful to auto

//        robot.subsystem.setPower(0.5);
//
//        robot.runForTime(2000);
//
//        robot.subsystem.setPower(0.3);
//
//        robot.subsystem.servoPositionOne();
//
//        robot.runUntil(() -> Math.toDegrees((float) robot.subsystem.imuSensor.getValue()) >= 10 );
//
//        robot.subsystem.servoPositionTwo();
//
//        robot.subsystem.motor.setPower(1);
//
//        robot.runUntilStop();

        }
    }
}
