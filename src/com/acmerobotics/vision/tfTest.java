package com.acmerobotics.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@TeleOp
public class tfTest extends LinearOpMode {

    @Override
    public void runOpMode(){

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry Telemetry = dashboard.getTelemetry();

        RingDetector ringDetector = new RingDetector(hardwareMap);

        waitForStart();

        ringDetector.startDetecting();

        while (!isStopRequested()){

            Telemetry.addData("rings", ringDetector.detectedRings());
            Telemetry.update();
        }
        ringDetector.stopDetecting();
    }
}
