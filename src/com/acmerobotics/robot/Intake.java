package com.acmerobotics.robot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.util.TeleOpActionImpl;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.acmerobotics.robomatic.util.StickyGamepad;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake extends Subsystem implements TeleOpActionImpl{

    private DcMotorEx intakeMotor;
    private Servo intakeServo;

    public static double upPosition = 0;
    public static double downPosition = 0;


    private int num = 0;

    public Intake(Robot robot){
        super("Intake");

        intakeMotor = robot.getMotor("intakeMotor");
        intakeServo = robot.getServo("intakeServo");
    }

    @Override
    public void update(Canvas overlay){
        telemetryData.addData("upPosition", upPosition);
        telemetryData.addData("downPosition", downPosition);
        telemetryData.addData("num", num);

    }

    @Override
    public void action(Gamepad gamepad1, Gamepad gamepad2, StickyGamepad stickyGamepad1, StickyGamepad stickyGamepad2){
        // press once to bring intake down and run wheels
        // press again to stop wheels and bring intake up

        if (stickyGamepad1.a) {
            intakeRings();
        }
    }


    public void intakeRings(){

        boolean runIntake = isOdd();

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


    private boolean isOdd(){
        num++;

        return  !(num % 2 == 0);
    }
}
