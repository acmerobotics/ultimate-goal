package com.acmerobotics.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.util.StickyGamepad;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TestV1 extends LinearOpMode {

    public int towerLevel = 3;

    @Override
    public void runOpMode(){
        ACMERobot robot = new ACMERobot(this);
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        while (!isStopRequested()) {

            // press a to start launcher motor, press a again to stop
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

            // adjust aim in small increments
            if (stickyGamepad.dpad_right){
                robot.launcher.adjustAimUp();
            }
            if (stickyGamepad.dpad_left){
                robot.launcher.adjustAimDown();
            }

            // change aim between towerLevels with dpad up and down
            if (stickyGamepad.dpad_up){
                towerLevel += 1;
                Range.clip(towerLevel, 1, 3);
                robot.launcher.setTowerLevel(towerLevel);
            }
            if (stickyGamepad.dpad_down){
                towerLevel -= 1;
                Range.clip(towerLevel, 1, 3);
                robot.launcher.setTowerLevel(towerLevel);
            }

            telemetry.update();
            stickyGamepad.update();
            robot.update();
        }

    }

}
