package com.acmerobotics.robot;

import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.vision.RingDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class ACMERobot extends Robot {

    public final Drive drive;
<<<<<<< HEAD
    public final Intake intake;
    public final RingDetector ringDetector;
    public final Launcher launcher; // adjust Launcher Version HERE!

    public double len = 17.4;
    public double width = 18;
    public double errorMargin = 4;
=======
    public final WobbleGoalSubSys wobbleGoal;
>>>>>>> e7f376718ecc8c780c29d5d34b44f56e4aa1cd81

    public ACMERobot(LinearOpMode opMode){
        super(opMode);

        registerHub("Expansion Hub 1");
        registerHub("Control Hub");

        drive = new Drive(this, opMode);
<<<<<<< HEAD
        intake = new Intake(this);
        ringDetector = new RingDetector(opMode.hardwareMap);
        launcher = new Launcher(this);

        registerSubsytem(drive);
        registerSubsytem(intake);
        registerSubsytem(ringDetector);
        registerSubsytem(launcher);
    }

    public void shootRingA(){
        launcher.kickRing();
        runForTime(1000);
        launcher.resetKicker();
        runForTime(1000);
=======
        wobbleGoal = new WobbleGoalSubSys(this);

        registerSubsytem(drive);
        registerSubsytem(wobbleGoal);
>>>>>>> e7f376718ecc8c780c29d5d34b44f56e4aa1cd81
    }
}
