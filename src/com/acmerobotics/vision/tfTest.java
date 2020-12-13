package com.acmerobotics.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@TeleOp
public class tfTest extends LinearOpMode {

    @Override
    public void runOpMode(){

        RingDetector ringDetector = new RingDetector();

        ringDetector.init(hardwareMap);

        waitForStart();

        ringDetector.startDetecting();

        while (!isStopRequested()){

            telemetry.addData("rings", ringDetector.detectedRings());
            telemetry.update();
        }
        ringDetector.stopDetecting();
    }
}
