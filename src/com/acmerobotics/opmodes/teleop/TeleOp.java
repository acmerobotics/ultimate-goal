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

    private boolean hold = false;
    private boolean lift = false;

    private boolean isSlowMode = false;

    @Override
    public void runOpMode(){

        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);

        ACMERobot robot = new ACMERobot(this);

        robot.launcher.aimServo.setPosition(0.6);

        waitForStart();

        while (!isStopRequested()) {

            if (!isSlowMode) {
                Pose2d v = new Pose2d(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
                robot.drive.setPower(v);
            }

            else{
                Pose2d v = new Pose2d(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
                robot.drive.setSlowPower(v);
            }

            if (stickyGamepad.y){
                isSlowMode = !isSlowMode;
            }

            // press once to bring intake down and run wheels
            // press again to stop wheels
            if (stickyGamepad.a) {
                robot.intake.intakeRings();
            }


            // press a to start launcher motor, press a again to stop
            if (stickyGamepad.x){
                robot.launcher.shoot();
            }

            // press trigger to kick ring and launcher, release to reset kicker
            if (gamepad1.right_trigger > 0.1){
                robot.launcher.kickRing();
            }
            if (gamepad1.right_trigger <= 0.1){
                robot.launcher.resetKicker();
            }

            // adjust aim to tower level
            if (stickyGamepad.dpad_right){
                robot.launcher.shootHigh();
            }
            if (stickyGamepad.dpad_left){
                robot.launcher.shootLow();
            }

            // adjust aim with power
            if (stickyGamepad.dpad_up){
                robot.launcher.adjustAimUp();
            }
            if (stickyGamepad.dpad_down){
                robot.launcher.adjustAimDown();
            }


            if (stickyGamepad.b){
                moveArm(robot);
            }
            if (stickyGamepad.right_bumper){
                grab(robot);
            }

            stickyGamepad.update();
            robot.update();
        }
    }


    private void grab(ACMERobot robot){
        hold = !hold;

        robot.wobbleGoal.wobbleGoalHand(hold);
    }

    private void moveArm(ACMERobot robot){
        lift = !lift;

        if (lift){
            robot.wobbleGoal.wobbleGoalArm(false, true);
        }

        else {
            robot.wobbleGoal.wobbleGoalArm(true, false);
        }
    }
}
