package com.acmerobotics.opmodes.auto.shoot;

import com.acmerobotics.opmodes.auto.Auto;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "shoot")
public class ShootRedB extends Auto {

    @Override
    public void run(){
        ACMERobot robot = new ACMERobot(this);

//        robot.drive.moveForward(80 -robot.len);
//        robot.runUntil(robot.drive::atYPosition);
//
//        robot.drive.strafeRight(12);
//        robot.runUntil(robot.drive::atStrafePosition);
//
//        robot.drive.stopMotors();
//        robot.update();
//
//        // shoot rings
//
//        robot.drive.moveForward(robot.len / 2);
//        robot.runUntil(robot.drive::atYPosition);


        robot.drive.stopMotors();
        robot.runUntilStop();

    }

}
