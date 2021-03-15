package com.acmerobotics.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.util.StickyGamepad;
import com.acmerobotics.robot.ACMERobot;
import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class testAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        ACMERobot robot = new ACMERobot(this);
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();


       // while (!isStopRequested()) {

//
//
//            if (stickyGamepad.a){
//                robot.drive.moveForward(48);
//
//            }
//
//            if (stickyGamepad.b){
//                robot.drive.moveBack(48);
//
//            }
//
//            if (stickyGamepad.x){
//                robot.drive.stopMotors();
//            }


//        robot.drive.moveForward(48);
//        robot.runUntil(robot.drive::atYPosition);
//
//        robot.drive.strafeRight(24);
//        robot.runUntil(robot.drive::atStrafePosition);
//
//        robot.drive.moveBack(48);
//        robot.runUntil(robot.drive::atYPosition);
//
//        robot.drive.strafeLeft(24);

//        robot.drive.automaticY(48);
//        robot.runUntil(robot.drive::atYPosition);

        robot.drive.automaticStrafe(24);
        robot.runUntil(robot.drive::atStrafePosition);

//        robot.drive.automaticY(-48);
//        robot.runUntil(robot.drive::atYPosition);
//
//        robot.drive.automaticStrafe(-24);
//        robot.runUntil(robot.drive::atStrafePosition);

//        robot.drive.moveForward(0);

        //robot.runUntil(robot.drive::atTurningPosition);

        //robot.drive.stopMotors();

        //robot.drive.stopVel();

//            // press a to start launcher motor, press a again to stoppastFirstCycle = true; // prevent the system from detecting a reached position before the movement started
//            if (stickyGamepad.x){
//                robot.launcher.shoot();
//            }


            //stickyGamepad.update();
            //robot.update();
        robot.runUntilStop();

        //}
    }

}
