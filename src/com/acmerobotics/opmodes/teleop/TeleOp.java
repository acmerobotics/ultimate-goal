package com.acmerobotics.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robot.ACMERobot;
import com.acmerobotics.robot.WobbleGoalSubSys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends LinearOpMode {

    public static double handPos = 0;
    public static double armPos = 0;

    @Override
    public void runOpMode(){

        ACMERobot robot = new ACMERobot(this);

        waitForStart();

        while (!isStopRequested()){
            robot.wobbleGoal.wobbleGoalHold.setPosition(handPos);
            robot.wobbleGoal.wobbleGoalLift.setPosition(armPos);

            robot.update();
        }

    }
}
