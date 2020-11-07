package com.acmerobotics.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.robot.ACMERobot;
import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name="Drive Test")
public class DriveTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        ACMERobot robot = new ACMERobot(this);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        while(!isStopRequested()){

            Pose2d v = new Pose2d(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            robot.drive.setPower(v);

            robot.update();

        }
    }
}
