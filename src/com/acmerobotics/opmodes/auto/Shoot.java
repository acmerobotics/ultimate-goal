package com.acmerobotics.opmodes.auto;

import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Shoot extends Auto {

    @Override
    public void runBlueA(){
        ACMERobot robot = new ACMERobot(this);

        robot.drive.moveForward(80 - robot.len);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.strafeRight(12);
        robot.runUntil(robot.drive::atStrafePosition);

        robot.drive.stopMotors();
        robot.update();

        // shoot rings

        robot.drive.moveForward(robot.len / 2);
        robot.runUntil(robot.drive::atYPosition);
    }

    @Override
    public void runBlueB(){
        ACMERobot robot = new ACMERobot(this);

        robot.drive.moveForward(80 - robot.len);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.strafeLeft(12);
        robot.runUntil(robot.drive::atStrafePosition);

        robot.drive.stopMotors();
        robot.update();

        // shoot rings

        robot.drive.moveForward(robot.len / 2);
        robot.runUntil(robot.drive::atYPosition);
    }

    @Override
    public void runRedB(){
        ACMERobot robot = new ACMERobot(this);

        robot.drive.moveForward(80 -robot.len);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.strafeRight(12);
        robot.runUntil(robot.drive::atStrafePosition);

        robot.drive.stopMotors();
        robot.update();

        // shoot rings

        robot.drive.moveForward(robot.len / 2);
        robot.runUntil(robot.drive::atYPosition);
    }

    @Override
    public void runRedA(){
        ACMERobot robot = new ACMERobot(this);

        robot.drive.moveForward(80 - robot.len);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.strafeLeft(12);
        robot.runUntil(robot.drive::atStrafePosition);

        robot.drive.stopMotors();
        robot.update();

        // shoot rings

        robot.drive.moveForward(robot.len / 2);
        robot.runUntil(robot.drive::atYPosition);
    }
}
