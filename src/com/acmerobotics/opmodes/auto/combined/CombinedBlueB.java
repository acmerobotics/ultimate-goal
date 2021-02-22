package com.acmerobotics.opmodes.auto.combined;

import com.acmerobotics.opmodes.auto.Auto;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Combined")
public class CombinedBlueB extends Auto {

    @Override
    public void run() throws InterruptedException {

        ACMERobot robot = new ACMERobot(this);

        // grab wobble
        robot.wobbleGoal.wobbleGoalHandLoose();
        robot.runForTime(500);


        // TO_RINGS
        robot.drive.moveForward(14);
        robot.runUntil(robot.drive::atYPosition);
        robot.drive.turnLeft(30);
        robot.runUntil(robot.drive::atTurningPosition);


        // DETECT_RINGS
        robot.drive.stopMotors();
        robot.ringDetector.startDetecting();
        robot.runForTime(1000);
        robot.ringDetector.stopDetecting();
        robot.update();
        telemetry.addData("rings detected", robot.ringDetector.detectedRings());
        telemetry.update();

        // TURN_BACK
        robot.drive.turnRight(30);
        robot.runUntil(robot.drive::atTurningPosition);

        // AVOID RING
        robot.drive.strafeRight(5);
        robot.runUntil(robot.drive::atStrafePosition);


        // MOVE_TO_LINE
        robot.drive.moveForward(34 + 14);
        robot.runUntil(robot.drive::atYPosition);

        // UNDO AVOID RING
        robot.drive.strafeLeft(5);
        robot.runUntil(robot.drive::atStrafePosition);


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
            robot.drive.strafeLeft(23);
            robot.runUntil(robot.drive::atStrafePosition);

            robot.drive.moveForward(20);
            robot.runUntil(robot.drive::atYPosition);

            robot.drive.turnLeft(55);
            robot.runUntil(robot.drive::atTurningPosition);

            // drop wobble
            robot.dropWobble();

            robot.drive.turnRight(55);
            robot.runUntil(robot.drive::atTurningPosition);
        }

        if (targetZone == TargetZone.B){
            robot.drive.moveForward(24 + 6);
            robot.runUntil(robot.drive::atYPosition);

            robot.drive.strafeLeft(6);
            robot.runUntil(robot.drive::atStrafePosition);

            robot.drive.turnLeft(35);
            robot.runUntil(robot.drive::atTurningPosition);

            // drop wobble
            robot.dropWobble();

            robot.drive.turnRight(35);
            robot.runUntil(robot.drive::atTurningPosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveForward(48 + 8); // 48 + 6
            robot.runUntil(robot.drive::atYPosition);

            robot.drive.strafeLeft(22); // check if it is more efficient to do a 180 turn + 180 turn or strafe 24 in
            robot.runUntil(robot.drive::atStrafePosition);

            robot.drive.turnLeft(45);
            robot.runUntil(robot.drive::atTurningPosition);

            // drop wobble
            robot.dropWobble();

            robot.drive.turnRight(45);
            robot.runUntil(robot.drive::atTurningPosition);
        }


        // MOVE_BEHIND_LINE (will probably need to add more to the move back amount to ensure the robot is behind line)
        if (targetZone == TargetZone.A){
            robot.drive.moveBack(24);
            robot.runUntil(robot.drive::atYPosition);
        }

        if (targetZone == TargetZone.B){
            robot.drive.moveBack(34);
            robot.runUntil(robot.drive::atYPosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveBack(54);
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
            robot.drive.moveForward(6);
            robot.runUntil(robot.drive::atYPosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.strafeRight(10);
            robot.runUntil(robot.drive::atStrafePosition);
        }


        // ADJUST_AIM
        robot.drive.turnTo(0);
        robot.runForTime(2000);


        // SHOOT_RINGS
        robot.launcher.shootHigh();
        robot.runUntil(robot.launcher::isMaxVelocity);
        robot.shootRingA();
        robot.shootRingA();
        robot.shootRingA();
        robot.launcher.shoot();
        robot.update();

       // PARK
        robot.drive.moveForward((robot.len / 2) + 6);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.stopMotors();
        robot.runUntilStop();
    }
}
