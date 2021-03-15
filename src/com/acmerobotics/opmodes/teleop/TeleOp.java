package com.acmerobotics.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.util.StickyGamepad;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends LinearOpMode {

    private boolean isSlowMode = false;
    private String shootingTarget = "High";

    boolean inAutomatic = false;

    boolean shooting = false;

    boolean ready = true;

    double shootingPos = 0;

    @Override
    public void runOpMode(){

        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);

        ACMERobot robot = new ACMERobot(this);

        robot.launcher.aimServo.setPosition(robot.launcher.servoAim);

        robot.intake.moveServo();

        robot.update();

        waitForStart();

        robot.intake.moveServo();

        robot.update();

        while (!isStopRequested()) {

            if (!inAutomatic) {
                if (!isSlowMode) {
                    Pose2d v = new Pose2d(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
                    robot.drive.setPower(v);
                } else {
                    Pose2d v = new Pose2d(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
                    robot.drive.setSlowPower(v);
                }
            }

            if (stickyGamepad.y){
                isSlowMode = !isSlowMode;
            }

            // press once to bring intake down and run wheels
            // press again to stop wheels
            if (stickyGamepad.a) {
                robot.intake.intakeRings();
            }
            // reverse power in case of stuck ring
            if (gamepad1.left_trigger > 0.1){
                robot.intake.reverseIntake(gamepad1.left_trigger);
            }


            // press a to start launcher motor, press a again to stop
            if (stickyGamepad.x){
                inAutomatic = true;

                robot.drive.autoTrajectory(robot.drive.omniTicksPerInch(((int)  shootingPos - robot.drive.omniTrackerX.getCurrentPosition())));

                ready = false;
            }
            if (inAutomatic && robot.drive.atStrafePositionAutomatic() && !shooting){

                robot.drive.stopVel();

                shooting = true;
                ready = true;

            }
            if (shooting){

//                robot.launcher.shootMid();
//
//                robot.runUntil(robot.launcher::isMaxVelocity);
//
//                robot.shootRingA();
//                robot.shootRingA();
//                robot.shootRingA();
//
//                robot.launcher.shoot();

                shooting = false;

                robot.drive.switchWheelDirections(true);

                inAutomatic = false;
            }

            if (stickyGamepad.left_bumper){
                shootingPos = robot.drive.omniTrackerX.getCurrentPosition();
                robot.drive.prepareMotors();
            }

            // press trigger to kick ring and launcher, release to reset kicker
            if (gamepad1.right_trigger > 0.1){
                robot.launcher.kickRing();
            }
            if (gamepad1.right_trigger <= 0.1){
                robot.launcher.resetKicker();
            }

//            // adjust aim to tower level
//            if (stickyGamepad.dpad_right){
//                robot.launcher.shootHigh();
//            }
            if (stickyGamepad.dpad_left){
                robot.wobbleGoal.wobbleHigh();
            }

            // adjust aim to tower level
            if (stickyGamepad.dpad_up){
                robot.launcher.shootHigh();
                shootingTarget = "High";
            }
            if (stickyGamepad.dpad_down){
                robot.launcher.shootMid();
                shootingTarget = "Mid";
            }

            if (stickyGamepad.dpad_right){
                robot.launcher.shoot();
            }


            if (stickyGamepad.b){
                robot.moveArm();
            }
            if (stickyGamepad.right_bumper){
                robot.grab();
            }

            telemetry.addData("shooting target", shootingTarget); // high or mid
            telemetry.addData("ready to shoot", ready);

            stickyGamepad.update();
            robot.update();
            telemetry.update();
        }
    }
}
