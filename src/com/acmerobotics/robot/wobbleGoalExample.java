package com.acmerobotics.robot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class wobbleGoalExample extends Subsystem {

    public Servo wobbleGoalLift;
    public Servo wobbleGoalRelease;

    private double armUp;
    private double armDown;
    private boolean isUp;

    private double holdWobble;
    private double releaseWobble;
    private boolean isClosed;

    private LinearOpMode opMode;

    public wobbleGoalExample(Robot robot, LinearOpMode opMode){
        super("wobbleGoalExample");

        this.opMode = opMode;

        wobbleGoalLift = robot.getServo("wobbleGoalLift");
        wobbleGoalRelease = robot.getServo("wobbleGoalRelease");

        armUp = 0;
        armDown = 1;

        isUp = true;

        releaseWobble = 1;
        holdWobble = 0;

        isClosed = true;

    }

    @Override
    public void update(Canvas fieldOverlay) {

        if(isUp = true){

            telemetryData.addData("Arm Is Up", null);

        } else {

            telemetryData.addData("Arm Is Down", null);

        }

        if(isClosed = true){

            telemetryData.addData("Hand Is Closed", null);

        } else {

            telemetryData.addData("Hand Is Open", null);

        }
    }

    public void wobbleGoalArm(boolean dPadDown, boolean dPadUp){

        if (dPadDown){

            wobbleGoalLift.setPosition(armDown);

        }

        if (dPadUp){

            wobbleGoalLift.setPosition(armUp);

        }

    }



    public void wobbleGoalHand(boolean a){

     //not done

    }

}
