package com.acmerobotics.robot;

import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestRobot extends Robot {
    public final TestSubsystem subsystem;

    public TestRobot(LinearOpMode opMode){
        super(opMode);

        registerHub("Expansion Hub 1");

        subsystem = new TestSubsystem(this, opMode.hardwareMap);
        registerSubsytem(subsystem);
    }
}


