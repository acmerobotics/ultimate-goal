package com.acmerobotics.robot;

import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestRobot extends Robot {

    public final TestSubsystem subsystem;

    public TestRobot(LinearOpMode opMode){
        super(opMode); // Robot constructor

        registerHub("Expansion Hub 1"); // will be used for hardwareMapping for hardwareDevice controller usage

        subsystem = new TestSubsystem(this, opMode.hardwareMap);
        registerSubsytem(subsystem); // put subsystem in list of subsystems so they will be updated every Robot update
    }
}


