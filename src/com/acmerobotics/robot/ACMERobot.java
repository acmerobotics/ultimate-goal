package com.acmerobotics.robot;

import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.vision.RingDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ACMERobot extends Robot {

    public final Drive drive;
    public final RingDetector ringDetector;

    public ACMERobot(LinearOpMode opMode){
        super(opMode);

        registerHub("Expansion Hub 2");

        drive = new Drive(this, opMode);
        registerSubsytem(drive);

        ringDetector = new RingDetector(opMode.hardwareMap);
    }
}
