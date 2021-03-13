package com.acmerobotics.opmodes.auto.combined;

import com.acmerobotics.opmodes.auto.Auto;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//    TO_RINGS,
//    TURN_TO_RINGS,
//    DETECT_RINGS,
//    TURN_BACK,
//    MOVE_TO_LINE,
// 180_TURN  (only red)
//    MOVE_TO_SQUARE,
//    STRAFE_TO_SQUARE,
//    MOVE_BEHIND_LINE,
//    STRAFE_TO_TOWER,
// 180_TURN  (only red)
//    SHOOT_RINGS,
//    PARK

@Autonomous(group = "Combined")
public class CombinedBlueA extends Auto {
    @Override
    public void run() throws InterruptedException {

        ACMERobot robot = new ACMERobot(this);

        robot.intake.moveServo();
        robot.update();

        // grab wobble
        robot.wobbleGoal.wobbleGoalHandLoose();
        robot.runForTime(500);

        // TO_RINGS
        robot.drive.moveForward(8);
        robot.runUntil(robot.drive::atYPosition);


        // DETECT_RINGS
        robot.drive.stopMotors();
        robot.ringDetector.startDetecting();
        robot.runForTime(1000);
        robot.ringDetector.stopDetecting();
        robot.update();
        telemetry.addData("rings detected", robot.ringDetector.detectedRings());
        telemetry.update();

        // AVOID RING
        robot.drive.strafeLeft(5);
        robot.runUntil(robot.drive::atStrafePosition);


        // MOVE_TO_LINE
        robot.drive.moveForward(48);
        robot.runUntil(robot.drive::atYPosition);

        // UNDO AVOID RING
        robot.drive.strafeRight(5);
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

            robot.drive.turnLeft(20);
            robot.runUntil(robot.drive::atTurningPosition);

            // drop wobble
            robot.dropWobble();

            robot.drive.turnRight(20);
            robot.runUntil(robot.drive::atTurningPosition);
        }

        if (targetZone == TargetZone.B){
            robot.drive.moveForward(23);
            robot.runUntil(robot.drive::atYPosition);

            robot.drive.turnRight(45);
            robot.runUntil(robot.drive::atTurningPosition);

            // drop wobble
            robot.dropWobble();

            robot.drive.turnLeft(45);
            robot.runUntil(robot.drive::atTurningPosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveForward(48 + 6); // 48
            robot.runUntil(robot.drive::atYPosition);

            robot.drive.turnLeft(40);
            robot.runUntil(robot.drive::atTurningPosition);

            // drop wobble
            robot.dropWobble();

            robot.drive.turnRight(40);
            robot.runUntil(robot.drive::atTurningPosition);
        }


         //MOVE_BEHIND_LINE (will probably need to add more to the move back amount to ensure the robot is behind line)
        if (targetZone == TargetZone.A){
            robot.drive.moveForward(4);
            robot.runUntil(robot.drive::atYPosition);
        }

        if (targetZone == TargetZone.B){
            robot.drive.moveBack(14);
            robot.runUntil(robot.drive::atYPosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveBack(63);
            robot.runUntil(robot.drive::atYPosition);
        }


        // STRAFE_TO_TOWER
        if (targetZone == TargetZone.A){
            robot.drive.strafeRight(15);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        if (targetZone == TargetZone.B){
            robot.drive.strafeRight(12);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.strafeRight(10);
            robot.runUntil(robot.drive::atStrafePosition);
        }


        // ADJUST_AIM
        robot.drive.turnTo(0);
        robot.runForTime(1000);

        // SHOOT_RINGS
        robot.launcher.shootHigh();
        robot.runUntil(robot.launcher::isMaxVelocity);
        robot.shootRingA();
        robot.shootRingA();
        robot.shootRingA();
        robot.launcher.shoot();
        robot.update();

        // PARK
        robot.drive.moveForward((robot.len / 2) + 4);
        robot.runUntil(robot.drive::atYPosition);

//        robot.drive.stopMotors();
        robot.runUntilStop();
    }
}
