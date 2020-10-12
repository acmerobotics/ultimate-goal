package com.acmerobotics.robot;

import com.acmerobotics.robomatic.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ACMERobot extends Robot {

    public final Drive drive;

    public double len = 0;
    public double width = 0;

    public ACMERobot(LinearOpMode opMode){
        super(opMode);

        registerHub("Expansion Hub 2");

        drive = new Drive(this, opMode);
        registerSubsytem(drive);
    }
}
