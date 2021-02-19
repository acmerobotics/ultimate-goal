package com.acmerobotics.robot;

import com.acmerobotics.robomatic.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class ACMERobot extends Robot {

    public final Drive drive;
    public final WobbleGoalSubSys wobbleGoal;

    public ACMERobot(LinearOpMode opMode){
        super(opMode);

        registerHub("Expansion Hub 1");
        registerHub("Control Hub");

        drive = new Drive(this, opMode);
        wobbleGoal = new WobbleGoalSubSys(this);

        registerSubsytem(drive);
        registerSubsytem(wobbleGoal);
    }
}
