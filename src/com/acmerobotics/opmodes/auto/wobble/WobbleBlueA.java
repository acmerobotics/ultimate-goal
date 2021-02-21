package com.acmerobotics.opmodes.auto.wobble;

import com.acmerobotics.opmodes.auto.Auto;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Wobble")
public class WobbleBlueA extends Auto {

    @Override
    public void run() throws InterruptedException {

        ACMERobot robot = new ACMERobot(this);

        // grab wobble
        robot.grab();
        robot.update();

        // TO_RINGS
        robot.drive.moveForward(14);
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
        robot.drive.moveForward(34 + 14);
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
            robot.drive.moveForward(8);
            robot.runUntil(robot.drive::atYPosition);

            robot.drive.turnLeft(40);
            robot.runUntil(robot.drive::atTurningPosition);

            // drop wobble
            robot.dropWobble();

            robot.drive.turnRight(40);
            robot.runUntil(robot.drive::atTurningPosition);
        }

        if (targetZone == TargetZone.B){
            robot.drive.moveForward(24 + 2);
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

            robot.drive.turnRight(25);
            robot.runUntil(robot.drive::atTurningPosition);
        }


        // PARK
        if (targetZone == TargetZone.A){
            robot.drive.moveForward(6);
            robot.runUntil(robot.drive::atYPosition);
        }

        if (targetZone == TargetZone.B){
            robot.drive.moveBack(10);
            robot.runUntil(robot.drive::atYPosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveBack(45);
            robot.runUntil(robot.drive::atYPosition);
        }


        robot.drive.stopMotors();
        robot.runUntilStop();
    }

    private double turnAngle(){
        // camera location on robot
        double Cx = 0;
        double Cy = 0;

        // distance from rings
        double Dx = 11.25;
        double Dy = 1;

        double hypot = Math.sqrt(Dx*Dx + Dy*Dy);

        double theta = Math.acos((hypot*hypot - Dx*Dx - Dy*Dy) / -2 * Dx * Dy);

        return theta;
    }
}
