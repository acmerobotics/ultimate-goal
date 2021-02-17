package com.acmerobotics.robot;

import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.vision.RingDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ACMERobot extends Robot {

    public final Drive drive;
    public final Intake intake;
    public final RingDetector ringDetector;
    public final Launcher launcher; // adjust Launcher Version HERE!

    public double len = 17.4;
    public double width = 18;
    public double errorMargin = 4;

    public ACMERobot(LinearOpMode opMode){
        super(opMode);

        registerHub("Expansion Hub 1");
        registerHub("Control Hub");

        drive = new Drive(this, opMode);
        intake = new Intake(this);
        ringDetector = new RingDetector(opMode.hardwareMap);
        launcher = new Launcher(this);

        registerSubsytem(drive);
        registerSubsytem(intake);
        registerSubsytem(ringDetector);
        registerSubsytem(launcher);
    }
}
