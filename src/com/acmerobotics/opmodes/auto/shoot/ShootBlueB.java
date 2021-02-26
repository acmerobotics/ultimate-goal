package com.acmerobotics.opmodes.auto.shoot;

import com.acmerobotics.opmodes.auto.Auto;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "shoot")
public class ShootBlueB extends Auto {

    @Override
    public void run() throws InterruptedException{

        ACMERobot robot = new ACMERobot(this);

        robot.intake.moveServo();
        robot.update();

        robot.drive.moveForward(80 - (robot.len + robot.errorMargin));
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.turnTo(0);
        robot.runForTime(2000);

        robot.drive.strafeLeft(12);
        robot.runUntil(robot.drive::atStrafePosition);

        robot.drive.moveForward(4);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.stopMotors();
        robot.update();

        // shoot rings
        robot.launcher.shootHigh();
        robot.runUntil(robot.launcher::isMaxVelocity);
        robot.shootRingA();
        robot.shootRingA();
        robot.shootRingA();
        robot.launcher.shoot();
        robot.update();

        robot.drive.moveForward((robot.len /2) + 2);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.stopMotors();
        robot.runUntilStop();
    }

}
