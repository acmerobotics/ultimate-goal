package com.acmerobotics.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.robomatic.util.StickyGamepad;
import com.acmerobotics.robot.ACMERobot;
import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.vision.vuforiaSubsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "testTeleOp")
public class TestTele extends LinearOpMode {

    public static double aimPosition = 0;
    //public static double kickerPosition = 0;

    public static double leftPowerMod = 0.05;
    public static double rightPowerMod = 0.05;


    @Override
    public void runOpMode(){

        ACMERobot robot = new ACMERobot(this);
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        robot.drive.switchWheelDirections(false);

        robot.drive.moveForward(24);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.switchWheelDirections(true);

        while (!isStopRequested()) {

            Pose2d v = new Pose2d(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            robot.drive.setPower(v);

            if (stickyGamepad.a){
                //robot.launcher.launcherServo.setPosition(aimPosition);
                robot.drive.autoShoot();
            }

            robot.drive.finishedTrajectory(telemetry);

            if (stickyGamepad.x){
                //robot.launcher.aimServo.setPosition(0.6);
                robot.drive.stopMotors();
            }

//            robot.drive.moveForward(84);
//
//            robot.runUntil(robot.drive::atYPosition);
//
//            robot.drive.moveBack(84);
//
//            robot.runUntil(robot.drive::atYPosition);
//
//            // press a to start launcher motor, press a again to stop
//            if (stickyGamepad.x){
//                robot.launcher.shoot();
//            }

            telemetry.update();

            stickyGamepad.update();
            robot.update();

        }

    }
}
