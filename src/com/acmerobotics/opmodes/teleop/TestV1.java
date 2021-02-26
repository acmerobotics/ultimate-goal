package com.acmerobotics.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.util.StickyGamepad;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class TestV1 extends LinearOpMode {

    public static double aimPosition = 0.75;


    @Override
    public void runOpMode(){
        ACMERobot robot = new ACMERobot(this);
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        robot.launcher.aimServo.setPosition(robot.launcher.servoAim);
        robot.update();

        waitForStart();

        while (!isStopRequested()) {

            // press a to start launcher motor, press a again to stop
            if (stickyGamepad.b){
                robot.launcher.aimServo.setPosition(aimPosition);
            }

            if (stickyGamepad.x){
                robot.launcher.shoot();
            }

            // press trigger to kick ring and launcher, release to reset kicker
            if (gamepad1.right_trigger > 0.1){
                robot.launcher.kickRing();
            }
            if (gamepad1.right_trigger <= 0.1){
                robot.launcher.resetKicker();
            }

            // adjust aim to tower level
            if (stickyGamepad.dpad_up){
                //robot.intake.moveServo(); //////////////////////////////
                robot.intake.intakeServo.setPosition(robot.intake.servoPosHold);
            }
            if (stickyGamepad.dpad_down){
                robot.launcher.shootMid();
            }

            //telemetry.update();
            stickyGamepad.update();
            robot.update();
        }

    }

}
