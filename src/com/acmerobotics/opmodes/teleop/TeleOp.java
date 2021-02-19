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

    @Override
    public void runOpMode(){

        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);

        ACMERobot robot = new ACMERobot(this);

        waitForStart();

        while (!isStopRequested()) {

            Pose2d v = new Pose2d(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            robot.drive.setPower(v);

            // press once to bring intake down and run wheels
            // press again to stop wheels and bring intake up
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

            robot.wobbleGoal.wobbleGoalArm(gamepad1.dpad_up, gamepad1.dpad_down);
            robot.wobbleGoal.wobbleGoalHand(gamepad1.a);

            stickyGamepad.update();
            robot.update();
        }
    }

    private void grab(){

    }

    private void lift(){

    }
}
