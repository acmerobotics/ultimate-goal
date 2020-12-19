package com.acmerobotics.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@TeleOp
public class tfTest extends LinearOpMode {

    @Override
    public void runOpMode(){

        ACMERobot robot = new ACMERobot(this);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry Telemetry = dashboard.getTelemetry();

        waitForStart();

        robot.ringDetector.startDetecting();

        while (!isStopRequested()){

            Telemetry.addData("rings", robot.ringDetector.detectedRings());
            Telemetry.update();
        }
        robot.ringDetector.stopDetecting();
    }
}
