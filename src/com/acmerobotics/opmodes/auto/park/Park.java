package com.acmerobotics.opmodes.auto.park;

import com.acmerobotics.robomatic.config.ConfigurationLoader;
import com.acmerobotics.robot.ACMERobot;
import com.acmerobotics.util.Configuration;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "Park")
public class Park extends LinearOpMode {

    @Override
    public void runOpMode(){

        ACMERobot robot = new ACMERobot(this);

        Configuration config = (Configuration) new ConfigurationLoader(hardwareMap.appContext).getConfig();

        waitForStart();

        // might add a strafe to get out of the way of the autos of others in the future

       robot.drive.moveForward(84 - (robot.len / 2));

       robot.runUntil(robot.drive::atYPosition);

        robot.drive.stopMotors();

        robot.runUntilStop();
    }
}
