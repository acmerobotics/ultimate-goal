package com.acmerobotics.opmodes.auto.shoot;

import com.acmerobotics.opmodes.auto.Auto;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "shoot")
public class ShootBlueA extends Auto {

    @Override
    public void run() throws InterruptedException {

        ACMERobot robot = new ACMERobot(this);

        robot.drive.moveForward(80 - (robot.len + robot.errorMargin));
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.turnTo(0);
        robot.runForTime(2000);

        robot.drive.strafeRight(12);
        robot.runUntil(robot.drive::atStrafePosition);

        robot.drive.moveBack(4);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.stopMotors();
        robot.update();

        // shoot rings
        Thread.sleep(3000);

        robot.drive.moveForward((robot.len /2) + 1);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.stopMotors();
        robot.runUntilStop();
    }
}
