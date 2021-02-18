package com.acmerobotics.opmodes.teleop;

import com.acmerobotics.robomatic.util.StickyGamepad;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class WobbleGoalTest extends LinearOpMode {

    public void runOpMode(){
        ACMERobot robot = new ACMERobot(this);
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);

        waitForStart();

        while(!isStopRequested()){

            robot.wobbleGoal.wobbleGoalArm(gamepad1.dpad_up, gamepad1.dpad_down);
            robot.wobbleGoal.wobbleGoalHand(stickyGamepad.a);

        }
    }

}
