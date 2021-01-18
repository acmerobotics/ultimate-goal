package com.acmerobotics.robot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.hardware.CachingDcMotorEx;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake extends Subsystem {

    private DcMotorEx intakeMotor;

    private boolean runIntake = false;

    public Intake(Robot robot){
        super("Intake");

        intakeMotor = robot.getMotor("intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void update(Canvas overlay){

    }


    public void intakeRings(){

        runIntake = !runIntake;

        if (runIntake){

            intakeMotor.setPower(1);
        }
        else{

            intakeMotor.setPower(0);
        }
    }

}
