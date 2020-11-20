package com.acmerobotics.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.robomatic.util.StickyGamepad;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "testTeleOp")
public class TestTele extends LinearOpMode {

    @Override
    public void runOpMode(){

        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class,  "motor1");
        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        while (!isStopRequested()) {


            telemetry.update();
            dashboardTelemetry.update();
            stickyGamepad1.update();
        }

    }
}
