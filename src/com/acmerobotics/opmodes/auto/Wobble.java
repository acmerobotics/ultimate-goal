package com.acmerobotics.opmodes.auto;

import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class Wobble extends Auto{

//    TO_RINGS,
//    TURN_TO_RINGS,
//    DETECT_RINGS,
//    TURN_BACK,
//    MOVE_TO_LINE,
    // 180_TURN   (only red)
//    MOVE_TO_SQUARE,
//    STRAFE_TO_SQUARE,
//    PARK
//    STRAFE_TO_TOWER,

    @Override
    public void runBlueA(){
        ACMERobot robot = new ACMERobot(this);

        // TO_RINGS
        robot.drive.moveForward(46 - robot.len);
        robot.runUntil(robot.drive::atYPosition);


        // TURN_TO_RINGS
        robot.drive.turnRight(turnAngle());
        robot.runUntil(robot.drive::atTurningPosition);


        // DETECT_RINGS
        // afwajfksja;fds;lf


        // TURN_BACK
        robot.drive.turnLeft(turnAngle());
        robot.runUntil(robot.drive::atTurningPosition);


        // MOVE_TO_LINE
        robot.drive.moveForward(34);
        robot.runUntil(robot.drive::atYPosition);


        // ALIGN_WOBBLE_WITH_LINE
        robot.drive.moveForward(0); // Rw, location of wobble from the front
        robot.runUntil(robot.drive::atYPosition);


        // MOVE_TO_SQUARE
        if (targetZone == TargetZone.A){
            // drop wobble
        }

        if (targetZone == TargetZone.B){
            robot.drive.moveForward(24);
            robot.runUntil(robot.drive::atYPosition);

            robot.drive.strafeRight(24); // check if it is more efficient to do a 180 turn + 180 turn or strafe 24 in
            robot.runUntil(robot.drive::atStrafePosition);

            // drop wobble
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveForward(48);
            robot.runUntil(robot.drive::atYPosition);

            // drop wobble
        }


        // PARK
        if (targetZone == TargetZone.A){

        }

        if (targetZone == TargetZone.B){
            robot.drive.moveBack(24);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveBack(48); //Rw
            robot.runUntil(robot.drive::atStrafePosition);
        }


        // STRAFE_TO_TOWER
        if (targetZone == TargetZone.A){
            robot.drive.strafeRight(12);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        if (targetZone == TargetZone.B){
            robot.drive.strafeLeft(12);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.strafeRight(12);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        robot.drive.stopMotors();

    }

    @Override
    public void runBlueB(){
        ACMERobot robot = new ACMERobot(this);

        // TO_RINGS
        robot.drive.moveForward(46 - robot.len);
        robot.runUntil(robot.drive::atYPosition);


        // TURN_TO_RINGS
        robot.drive.turnLeft(turnAngle());
        robot.runUntil(robot.drive::atTurningPosition);


        // DETECT_RINGS
        // afwajfksja;fds;lf


        // TURN_BACK
        robot.drive.turnRight(turnAngle());
        robot.runUntil(robot.drive::atTurningPosition);


        // MOVE_TO_Line
        robot.drive.moveForward(34);
        robot.runUntil(robot.drive::atYPosition);


        // ALIGN_WOBBLE_WITH_LINE
        robot.drive.moveForward(0); // Rw, location of wobble from the front
        robot.runUntil(robot.drive::atYPosition);


        // MOVE_TO_SQUARE
        if (targetZone == TargetZone.A){
            robot.drive.strafeLeft(24); // check if it is more efficient to do a 180 turn + 180 turn or strafe 24 in
            robot.runUntil(robot.drive::atStrafePosition);

            // drop wobble
        }

        if (targetZone == TargetZone.B){
            robot.drive.moveForward(24);
            robot.runUntil(robot.drive::atYPosition);

            // drop wobble
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveForward(48);
            robot.runUntil(robot.drive::atYPosition);

            robot.drive.strafeLeft(24); // check if it is more efficient to do a 180 turn + 180 turn or strafe 24 in
            robot.runUntil(robot.drive::atStrafePosition);

            // drop wobble
        }


        // PARK
        if (targetZone == TargetZone.A){

        }

        if (targetZone == TargetZone.B){
            robot.drive.moveBack(24);
            robot.runUntil(robot.drive::atYPosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveBack(48); //Rw
            robot.runUntil(robot.drive::atYPosition);
        }


        // STRAFE_TO_TOWER
        if (targetZone == TargetZone.A){
            robot.drive.strafeRight(12);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        if (targetZone == TargetZone.B){
            robot.drive.strafeLeft(12);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.strafeRight(12);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        robot.drive.stopMotors();
    }

    @Override
    public void runRedB(){
        ACMERobot robot = new ACMERobot(this);

        // TO_RINGS
        robot.drive.moveForward(46 - robot.len);
        robot.runUntil(robot.drive::atYPosition);


        // TURN_TO_RINGS
        robot.drive.turnRight(turnAngle());
        robot.runUntil(robot.drive::atTurningPosition);


        // DETECT_RINGS
        // afwajfksja;fds;lf


        // TURN_BACK
        robot.drive.turnLeft(turnAngle());
        robot.runUntil(robot.drive::atTurningPosition);


        // MOVE_TO_Line
        robot.drive.moveForward(34);
        robot.runUntil(robot.drive::atYPosition);


        // ALIGN_WOBBLE_WITH_LINE
        robot.drive.moveForward(0); // Rw, location of wobble from the front
        robot.runUntil(robot.drive::atYPosition);


        // 180_TURN
        robot.drive.turnRight(180);
        robot.runUntil(robot.drive::atTurningPosition);


        // MOVE_TO_SQUARE
        if (targetZone == TargetZone.A){
            robot.drive.strafeLeft(24); // check if it is more efficient to do a 180 turn + 180 turn or strafe 24 in
            robot.runUntil(robot.drive::atStrafePosition);

            // drop wobble
        }

        if (targetZone == TargetZone.B){
            robot.drive.moveBack(24);
            robot.runUntil(robot.drive::atYPosition);

            // drop wobble
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveBack(48);
            robot.runUntil(robot.drive::atYPosition);

            robot.drive.strafeLeft(24); // check if it is more efficient to do a 180 turn + 180 turn or strafe 24 in
            robot.runUntil(robot.drive::atStrafePosition);

            // drop wobble
        }


        // PARK
        if (targetZone == TargetZone.A){

        }

        if (targetZone == TargetZone.B){
            robot.drive.moveForward(24);
            robot.runUntil(robot.drive::atYPosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveForward(48); //Rw
            robot.runUntil(robot.drive::atYPosition);
        }


        // STRAFE_TO_TOWER
        if (targetZone == TargetZone.A){
            robot.drive.strafeRight(12);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        if (targetZone == TargetZone.B){
            robot.drive.strafeLeft(12);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.strafeRight(12);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        robot.drive.stopMotors();
    }


    @Override
    public void runRedA(){
        ACMERobot robot = new ACMERobot(this);

        // TO_RINGS
        robot.drive.moveForward(46 - robot.len);
        robot.runUntil(robot.drive::atYPosition);


        // TURN_TO_RINGS
        robot.drive.turnLeft(turnAngle());
        robot.runUntil(robot.drive::atTurningPosition);


        // DETECT_RINGS
        // afwajfksja;fds;lf


        // TURN_BACK
        robot.drive.turnRight(turnAngle());
        robot.runUntil(robot.drive::atTurningPosition);


        // MOVE_TO_Line
        robot.drive.moveForward(34);
        robot.runUntil(robot.drive::atYPosition);


        // ALIGN_WOBBLE_WITH_LINE
        robot.drive.moveForward(0); // Rw, location of wobble from the front
        robot.runUntil(robot.drive::atYPosition);


        // 180_TURN
        robot.drive.turnRight(180);
        robot.runUntil(robot.drive::atTurningPosition);


        // MOVE_TO_SQUARE
        if (targetZone == TargetZone.A){
            // drop wobble
        }

        if (targetZone == TargetZone.B){
            robot.drive.moveBack(24);
            robot.runUntil(robot.drive::atYPosition);

            robot.drive.strafeRight(24); // check if it is more efficient to do a 180 turn + 180 turn or strafe 24 in
            robot.runUntil(robot.drive::atStrafePosition);

            // drop wobble
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveBack(48);
            robot.runUntil(robot.drive::atYPosition);

            // drop wobble
        }


        // PARK
        if (targetZone == TargetZone.A){

        }

        if (targetZone == TargetZone.B){
            robot.drive.moveBack(24);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.moveBack(48); //Rw
            robot.runUntil(robot.drive::atStrafePosition);
        }


        // STRAFE_TO_TOWER
        if (targetZone == TargetZone.A){
            robot.drive.strafeRight(12);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        if (targetZone == TargetZone.B){
            robot.drive.strafeLeft(12);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        if (targetZone == TargetZone.C){
            robot.drive.strafeRight(12);
            robot.runUntil(robot.drive::atStrafePosition);
        }

        robot.drive.stopMotors();
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
