package com.acmerobotics.opmodes.auto.ultimate;

import com.acmerobotics.opmodes.auto.Auto;
import com.acmerobotics.robot.ACMERobot;

public class UltimateBlueA extends Auto {

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


       // robot.drive.moveForward();
        // move forward a a bit

        // shoot rings

        // intake new rings if there are any

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

        if (targetZone == TargetZone.A){

        }

        if (targetZone == TargetZone.B){

        }

        if (targetZone == TargetZone.C){

        }



//    // AVOID RING
//        robot.drive.strafeLeft(5);
//        robot.runUntil(robot.drive::atStrafePosition);
//
//
//    // MOVE_TO_LINE
//        robot.drive.moveForward(48);
//        robot.runUntil(robot.drive::atYPosition);
//
//    // UNDO AVOID RING
//        robot.drive.strafeRight(6);
//        robot.runUntil(robot.drive::atStrafePosition);
//
//
//    // DETERMINE_TARGET_ZONE
//        if (robot.ringDetector.detectedRings() == 0){
//        targetZone = TargetZone.A;
//    }
//
//        else if (robot.ringDetector.detectedRings() == 1){
//        targetZone = TargetZone.B;
//    }
//
//        else{
//        targetZone = TargetZone.C;
//    }
//
//
//    // MOVE_TO_SQUARE
//        if (targetZone == TargetZone.A){
//
//        robot.drive.turnLeft(20);
//        robot.runUntil(robot.drive::atTurningPosition);
//
//        // drop wobble
//        robot.dropWobble();
//
//        robot.drive.turnRight(20);
//        robot.runUntil(robot.drive::atTurningPosition);
//    }
//
//        if (targetZone == TargetZone.B){
//        robot.drive.moveForward(23);
//        robot.runUntil(robot.drive::atYPosition);
//
//        robot.drive.turnRight(45);
//        robot.runUntil(robot.drive::atTurningPosition);
//
//        // drop wobble
//        robot.dropWobble();
//
//        robot.drive.turnLeft(45);
//        robot.runUntil(robot.drive::atTurningPosition);
//    }
//
//        if (targetZone == TargetZone.C){
//        robot.drive.moveForward(48 + 6); // 48
//        robot.runUntil(robot.drive::atYPosition);
//
//        robot.drive.turnLeft(40);
//        robot.runUntil(robot.drive::atTurningPosition);
//
//        // drop wobble
//        robot.dropWobble();
//
//        robot.drive.turnRight(40);
//        robot.runUntil(robot.drive::atTurningPosition);
//    }
//
//
//    //MOVE_BEHIND_LINE (will probably need to add more to the move back amount to ensure the robot is behind line)
//        if (targetZone == TargetZone.A){
//        robot.drive.moveForward(4);
//        robot.runUntil(robot.drive::atYPosition);
//    }
//
//        if (targetZone == TargetZone.B){
//        robot.drive.moveBack(14);
//        robot.runUntil(robot.drive::atYPosition);
//    }
//
//        if (targetZone == TargetZone.C){
//        robot.drive.moveBack(55);
//        robot.runUntil(robot.drive::atYPosition);
//    }
//
//
//    // STRAFE_TO_TOWER
//        if (targetZone == TargetZone.A){
//        robot.drive.strafeRight(15);
//        robot.runUntil(robot.drive::atStrafePosition);
//    }
//
//        if (targetZone == TargetZone.B){
//        robot.drive.strafeRight(12);
//        robot.runUntil(robot.drive::atStrafePosition);
//    }
//
//        if (targetZone == TargetZone.C){
//        robot.drive.strafeRight(10);
//        robot.runUntil(robot.drive::atStrafePosition);
//    }
//
//
//    // ADJUST_AIM
//        robot.drive.turnTo(0);
//        robot.runForTime(1000);
//
//    // SHOOT_RINGS
//        robot.launcher.shootHigh();
//        robot.runUntil(robot.launcher::isMaxVelocity);
//        robot.shootRingA();
//        robot.shootRingA();
//        robot.shootRingA();
//        robot.launcher.shoot();
//        robot.update();
//
//    // PARK
//        robot.drive.moveForward((robot.len / 2) + 4);
//        robot.runUntil(robot.drive::atYPosition);

        robot.drive.stopMotors();
        robot.runUntilStop();
}

}
