package com.acmerobotics.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.robomatic.util.StickyGamepad;
import com.acmerobotics.robot.ACMERobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="OneControllerTeleOp")
public class OneControllerTeleOp extends LinearOpMode {

    private int state;

    @Override
    public void runOpMode() throws InterruptedException {

        state = 1;

        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1);

        ACMERobot robot = new ACMERobot(this);


        waitForStart();


        while(!isStopRequested()){

            switch (state){

                case 1:

                    Pose2d v = new Pose2d(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
                    robot.drive.setPower(v);

                    // press once to bring intake down and run wheels
                    // press again to stop wheels and bring intake up
                    if (stickyGamepad1.a) {
                        robot.intake.intakeRings();
                    }

                    //other controls for gamepad 1

                    if (stickyGamepad1.x){

                        state = 2;
                    }

                    break;


                case 2:

                    //controls for gamepad 2 (we don't have those yet)

                    if (stickyGamepad1.x){
                        state = 1;
                    }



            }

            telemetry.addData("Gamepad State:", state);


            stickyGamepad1.update();
            robot.update();

            telemetry.update();


        }
    }
}
