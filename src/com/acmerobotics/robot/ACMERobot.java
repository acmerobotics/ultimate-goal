package com.acmerobotics.robot;

import com.acmerobotics.robomatic.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ACMERobot extends Robot {

    public final Drive drive;
    public final Intake intake;

    public ACMERobot(LinearOpMode opMode){
        super(opMode);

        //registerHub("Expansion Hub 1");
        registerHub("Expansion Hub 2");

        drive = new Drive(this, opMode);
        intake = new Intake(this);

        registerSubsytem(drive);
        registerSubsytem(intake);
    }
}
