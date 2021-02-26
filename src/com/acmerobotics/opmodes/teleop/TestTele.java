package com.acmerobotics.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.util.StickyGamepad;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Disabled
@TeleOp(name = "testTeleOp")
public class TestTele extends LinearOpMode {

    public static double aimPosition = 0;
    //public static double kickerPosition = 0;

    @Override
    public void runOpMode(){

        ACMERobot robot = new ACMERobot(this);
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        while (!isStopRequested()) {


//            if (stickyGamepad.a){
//                robot.launcher.launcherServo.setPosition(aimPosition);
//            }

            if (stickyGamepad.b){
                robot.launcher.aimServo.setPosition(0.6);
            }

            // press a to start launcher motor, press a again to stop
            if (stickyGamepad.x){
                robot.launcher.shoot();
            }


            stickyGamepad.update();
            robot.update();
        }

    }
}
