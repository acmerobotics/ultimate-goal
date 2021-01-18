package com.acmerobotics.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.robomatic.config.ConfigurationLoader;
import com.acmerobotics.robot.ACMERobot;
import com.acmerobotics.util.Configuration;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class TestAuto extends Auto{

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

    enum TargetZone {
        A,
        B,
        C
    }
    TargetZone targetZone = TargetZone.B;

    @Override
    public void runBlueA() {
        ACMERobot robot = new ACMERobot(this);

        robot.drive.strafeRight(12);
        robot.runUntil(robot.drive::atStrafePosition);

        robot.drive.moveForward(12);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.stopMotors();

        robot.runUntilStop();
    }

    @Override
    public void runBlueB(){
        ACMERobot robot = new ACMERobot(this);

        robot.drive.moveForward(12);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.turnRight(90);
        robot.runUntil(robot.drive::atTurningPosition);


        robot.drive.turnLeft(90);
        robot.runUntil(robot.drive::atTurningPosition);

        robot.drive.stopMotors();

        robot.runUntilStop();
    }

    @Override
    public void runRedB(){
        ACMERobot robot = new ACMERobot(this);

        robot.drive.moveForward(12);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.turnRight(90);
        robot.runUntil(robot.drive::atTurningPosition);

        robot.drive.stopMotors();

        robot.runUntilStop();
    }

    @Override
    public void runRedA(){
        ACMERobot robot = new ACMERobot(this);

        robot.drive.moveForward(12);
        robot.runUntil(robot.drive::atYPosition);
        telemetry.addData("m0 pos", robot.drive.motors[0].getCurrentPosition());
        telemetry.addData("target", robot.drive.target);

        robot.drive.turnRight(90);
        robot.runUntil(robot.drive::atTurningPosition);
        telemetry.addData("m0 pos", robot.drive.motors[0].getCurrentPosition());

        telemetry.addData("m0 pos", robot.drive.motors[0].getCurrentPosition());

        robot.drive.moveForward(12);
        robot.runUntil(robot.drive::atYPosition);

        robot.drive.stopMotors();

        telemetry.addData("m0 pos", robot.drive.motors[0].getCurrentPosition());
        telemetry.addData("target", robot.drive.target);
        telemetry.update();
        robot.runUntilStop();
    }

}
