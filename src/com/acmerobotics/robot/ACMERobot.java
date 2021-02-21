package com.acmerobotics.robot;

import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.vision.RingDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class ACMERobot extends Robot {

    public final Drive drive;
    public final Intake intake;
    public final RingDetector ringDetector;
    public final Launcher launcher; // adjust Launcher Version HERE!
    public final WobbleGoalSubSys wobbleGoal;


    public double len = 17.4;
    public double width = 18;
    public double errorMargin = 4;

    private boolean hold = false;
    private boolean lift = false;

    public ACMERobot(LinearOpMode opMode){
        super(opMode);

        registerHub("Expansion Hub 1");
        registerHub("Control Hub");

        drive = new Drive(this, opMode);
        intake = new Intake(this);
        ringDetector = new RingDetector(opMode.hardwareMap);
        launcher = new Launcher(this);
        wobbleGoal = new WobbleGoalSubSys(this);

        registerSubsytem(drive);
        registerSubsytem(intake);
        registerSubsytem(ringDetector);
        registerSubsytem(launcher);
        registerSubsytem(wobbleGoal);
    }

    public void shootRingA(){
        launcher.kickRing();
        runForTime(1000);
        launcher.resetKicker();
        runForTime(1000);

    }

    public void grab(){
        hold = !hold;

        wobbleGoal.wobbleGoalHand(hold);
    }

    public void moveArm(){
        lift = !lift;

        if (lift){
            wobbleGoal.wobbleGoalArm(false, true);
        }

        else {
            wobbleGoal.wobbleGoalArm(true, false);
        }
    }

    public void dropWobble(){
        drive.stopMotors();
        update();
        moveArm();
        runForTime(1000);
        grab();
        runForTime(1000);
        drive.moveBack(6);
        runUntil(drive::atYPosition);
        drive.stopMotors();
        update();
        moveArm();
        runForTime(1000);
        drive.moveForward(6);
        runUntil(drive::atYPosition);
        drive.stopMotors();
    }
}
