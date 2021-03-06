package com.acmerobotics.opmodes.auto.wobble;

import com.acmerobotics.opmodes.auto.Auto;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(group = "Wobble")
public class WobbleRedA extends Auto {
    @Override
    public void run() throws InterruptedException{

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
            robot.drive.turnRight(45);
            robot.runUntil(robot.drive::atTurningPosition);

            // drop wobble
            robot.drive.stopMotors();
            robot.runForTime(3000);

            robot.drive.turnLeft(45);
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

            robot.drive.turnRight(45);
            robot.runUntil(robot.drive::atTurningPosition);

            // drop wobble
            robot.drive.stopMotors();
            robot.runForTime(3000);

            robot.drive.turnLeft(45);
            robot.runUntil(robot.drive::atTurningPosition);
        }


        // PARK
        if (targetZone == TargetZone.A){
            robot.drive.moveForward(18);
            robot.runUntil(robot.drive::atYPosition);
        }

        if (targetZone == TargetZone.B){
            robot.drive.moveBack(18 - 4);
            robot.runUntil(robot.drive::atYPosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveBack(48 - 18);
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
