package com.acmerobotics.robot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.hardware.CachingDcMotorEx;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake extends Subsystem {

    private DcMotorEx intakeMotor;
    private Servo intakeServo;

    public static double upPosition = 0;
    public static double downPosition = 0;

    private boolean runIntake = false;

    public Intake(Robot robot){
        super("Intake");

        intakeMotor = robot.getMotor("intakeMotor");
        intakeServo = robot.getServo("intakeServo");
    }

    @Override
    public void update(Canvas overlay){
        telemetryData.addData("upPosition", upPosition);
        telemetryData.addData("downPosition", downPosition);

    }


    public void intakeRings(){

        runIntake = !runIntake;

        if (runIntake){

            dropIntake(true);
            intakeMotor.setPower(1);
        }
        else{

            dropIntake(false);
            intakeMotor.setPower(0);
        }
    }


    private void dropIntake(boolean lower){
        if (lower){
            intakeServo.setPosition(downPosition);
        }

        else{
            intakeServo.setPosition(upPosition);
        }
    }

}
