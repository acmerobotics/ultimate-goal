package com.acmerobotics.opmodes.auto.powershot;

import com.acmerobotics.opmodes.auto.Auto;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "power shot")
public class PowerShotBlueB extends Auto {
    ACMERobot robot = new ACMERobot(this);

    @Override
    public void run() throws InterruptedException {

        //robot.intake.moveServo();
       // robot.update();

        //move into positon
        robot.drive.moveForward(80 - (robot.len + robot.errorMargin));
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.turnTo(0);
        robot.runForTime(2000);

        robot.drive.strafeRight(35); //haha this probably isn't right
        robot.runUntil(robot.drive::atStrafePosition);

        robot.drive.moveBack(4);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.stopMotors();
        robot.update();

        //shoot first ring
        robot.launcher.shootMid();
        robot.runUntil(robot.launcher::isMaxVelocity);
        robot.shootRingA();

        //move to next power shot
        robot.drive.turnTo(0);
        robot.runForTime(2000);

        robot.drive.strafeRight(10);
        robot.runUntil(robot.drive::atStrafePosition);

        robot.drive.moveBack(4);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.stopMotors();
        robot.update();

        //shoot second ring
        robot.launcher.shootMid();
        robot.runUntil(robot.launcher::isMaxVelocity);
        robot.shootRingA();

        //move to final power shot
        robot.drive.turnTo(0);
        robot.runForTime(2000);

        robot.drive.strafeRight(10);
        robot.runUntil(robot.drive::atStrafePosition);

        robot.drive.moveBack(4);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.stopMotors();
        robot.update();

        //shoot final ring
        robot.launcher.shootMid();
        robot.runUntil(robot.launcher::isMaxVelocity);
        robot.shootRingA();

        //park
        robot.drive.moveForward(2);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.stopMotors();
        robot.runUntilStop();
    }
}
