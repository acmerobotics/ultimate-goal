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

        // TO_RINGS
        robot.drive.moveForward(46 - robot.len);
        robot.runUntil(robot.drive::atYPosition);

        // TURN_TO_RINGS
        robot.drive.turnRight(90);
        robot.runUntil(robot.drive::atTurningPosition);

        // DETECT_RINGS
        robot.drive.stopMotors();
        robot.runForTime(3000);

        // TURN_BACK
        robot.drive.turnLeft(90);
        robot.runUntil(robot.drive::atTurningPosition);

        // MOVE_TO_Line
        robot.drive.moveForward(34);
        robot.runUntil(robot.drive::atYPosition);

        // MOVE_TO_SQUARE
        if (targetZone == TargetZone.A){
            robot.drive.moveForward(8);
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
            robot.drive.moveForward(24 + 8);
            robot.runUntil(robot.drive::atYPosition);

            robot.drive.turnRight(45);
            robot.runUntil(robot.drive::atTurningPosition);

            // drop wobble
            robot.drive.stopMotors();
            robot.runForTime(3000);

            robot.drive.turnLeft(45);
            robot.runUntil(robot.drive::atTurningPosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveForward(48 + 6); // 48
            robot.runUntil(robot.drive::atYPosition);

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
            robot.drive.moveBack(15);
            robot.runUntil(robot.drive::atYPosition);
        }

        if (targetZone == TargetZone.B){
            robot.drive.moveBack(18);
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
            robot.drive.strafeRight(10);
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
