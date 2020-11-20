package com.acmerobotics.robot;

import com.acmerobotics.robomatic.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ACMERobot extends Robot {

    public final Drive drive;
    public final Intake intake;

    public ACMERobot(LinearOpMode opMode, boolean inTeleOp){
        super(opMode, inTeleOp);

        //registerHub("Expansion Hub 1");
        registerHub("Expansion Hub 2");

        drive = new Drive(this, inTeleOp);
        intake = new Intake(this);

        // register subsystems
        registerSubsytem(drive);
        registerSubsytem(intake);


        // register teleOp Actions
        registerTeleOpAction(intake);
    }
}
