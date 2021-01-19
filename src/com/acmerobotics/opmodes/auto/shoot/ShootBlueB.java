package com.acmerobotics.opmodes.auto.shoot;

import com.acmerobotics.opmodes.auto.Auto;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "shoot")
public class ShootBlueB extends Auto {

    @Override
    public void run() {

//        ACMERobot robot = new ACMERobot(this);
//
//        robot.drive.moveForward(80 - robot.len);
//        robot.runUntil(robot.drive::atYPosition);
//
//        robot.drive.strafeLeft(12);
//        robot.runUntil(robot.drive::atStrafePosition);
//
//        robot.drive.stopMotors();
//        robot.update();
//
//        // shoot rings
//
//        robot.drive.moveForward(robot.len / 2);
//        robot.runUntil(robot.drive::atYPosition);



        ACMERobot robot = new ACMERobot(this);

        robot.drive.moveForward(80-robot.len);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.turnTo(0);
        robot.runForTime(3000);

        robot.drive.stopMotors();
        robot.runUntilStop();
    }

}
