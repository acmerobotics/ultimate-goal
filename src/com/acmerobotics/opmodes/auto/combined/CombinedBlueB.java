package com.acmerobotics.opmodes.auto.combined;

import com.acmerobotics.opmodes.auto.Auto;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Combined")
public class CombinedBlueB extends Auto {

    @Override
    public void run() throws InterruptedException {

        ACMERobot robot = new ACMERobot(this);

        // TO_RINGS
        robot.drive.moveForward(46 - robot.len);
        robot.runUntil(robot.drive::atYPosition);

        // TURN_TO_RINGS
        robot.drive.turnLeft(90);
        robot.runUntil(robot.drive::atTurningPosition);

        // DETECT_RINGS
        robot.drive.stopMotors();
        robot.ringDetector.startDetecting();
        robot.runForTime(3000);
        robot.ringDetector.stopDetecting();
        robot.runForTime(500);

        // TURN_BACK
        robot.drive.turnRight(90);
        robot.runUntil(robot.drive::atTurningPosition);

        // MOVE_TO_LINE
        robot.drive.moveForward(34);
        robot.runUntil(robot.drive::atYPosition);


        // DETERMINE_TARGET_ZONE
        if (robot.ringDetector.detectedRings() == 0){
            targetZone = TargetZone.A;
        }

        else if (robot.ringDetector.detectedRings() == 1){
            targetZone = TargetZone.B;
        }

        else{
            targetZone = TargetZone.C;
        }


        // MOVE_TO_SQUARE
        if (targetZone == TargetZone.A){
            robot.drive.strafeLeft(24); // check if it is more efficient to do a 180 turn + 180 turn or strafe 24 in
            robot.runUntil(robot.drive::atStrafePosition);

            robot.drive.moveForward(24);
            robot.runUntil(robot.drive::atYPosition);

            robot.drive.turnLeft(45);
            robot.runUntil(robot.drive::atTurningPosition);

            // drop wobble
            robot.drive.stopMotors();
            robot.runForTime(3000);

            robot.drive.turnRight(45);
            robot.runUntil(robot.drive::atTurningPosition);
        }

        if (targetZone == TargetZone.B){
            robot.drive.moveForward(24 + 6);
            robot.runUntil(robot.drive::atYPosition);

            robot.drive.turnLeft(45);
            robot.runUntil(robot.drive::atTurningPosition);

            // drop wobble
            robot.drive.stopMotors();
            robot.runForTime(3000);

            robot.drive.turnRight(45);
            robot.runUntil(robot.drive::atTurningPosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveForward(48 + 6);
            robot.runUntil(robot.drive::atYPosition);

            robot.drive.strafeLeft(22); // check if it is more efficient to do a 180 turn + 180 turn or strafe 24 in
            robot.runUntil(robot.drive::atStrafePosition);

            robot.drive.turnLeft(45);
            robot.runUntil(robot.drive::atTurningPosition);

            // drop wobble
            robot.drive.stopMotors();
            robot.runForTime(3000);

            robot.drive.turnRight(45);
            robot.runUntil(robot.drive::atTurningPosition);
        }


        // MOVE_BEHIND_LINE (will probably need to add more to the move back amount to ensure the robot is behind line)
        if (targetZone == TargetZone.A){
            robot.drive.moveBack(24);
            robot.runUntil(robot.drive::atYPosition);
        }

        if (targetZone == TargetZone.B){
            robot.drive.moveBack(26);
            robot.runUntil(robot.drive::atYPosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveBack(48 - 6);
            robot.runUntil(robot.drive::atYPosition);
        }


        // STRAFE_TO_TOWER
        if (targetZone == TargetZone.A){
            robot.drive.strafeRight(10);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        if (targetZone == TargetZone.B){
            robot.drive.strafeLeft(10);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.strafeRight(10);
            robot.runUntil(robot.drive::atStrafePosition);
        }


        // ADJUST_AIM
        robot.drive.turnTo(0);
        robot.runForTime(2000);


        // SHOOT_RINGS
        robot.drive.stopMotors();
        robot.runForTime(3000);

        // PARK
        robot.drive.moveForward((robot.len / 2) + 1);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.stopMotors();
        robot.runUntilStop();
    }
}
