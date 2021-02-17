package com.acmerobotics.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.util.StickyGamepad;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestV1 extends LinearOpMode {

    @Override
    public void runOpMode(){
        ACMERobot robot = new ACMERobot(this);
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        while (!isStopRequested()) {

            if (stickyGamepad.a){
                robot.launcher.shoot();
            }

            if (stickyGamepad.dpad_down){
                robot.launcher.adjustAimDown();
            }

            if (stickyGamepad.dpad_up){
                robot.launcher.adjustAimUp();
            }

            telemetry.update();
            stickyGamepad.update();
            robot.update();
        }

    }

}
