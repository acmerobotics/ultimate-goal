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

    public Servo intakeServo;

    private boolean runIntake = false;

    public static double servoPosRelease = 0.4;
    public static double servoPosHold = 0.2;
    private boolean releaseServo = true;

    public Intake(Robot robot){
        super("Intake");

        intakeMotor = robot.getMotor("intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeServo = robot.getServo("intakeServo");
    }

    @Override
    public void update(Canvas overlay){
        telemetryData.addData("motor power", intakeMotor.getPower());
        telemetryData.addData("servo pos", intakeServo.getPosition());

    }

    public void reverseIntake(double power){
        intakeMotor.setPower(-power);
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

    public void moveServo(){
        releaseServo = !releaseServo;

        if (releaseServo){

            intakeServo.setPosition(servoPosRelease);
        }
        else{

            intakeServo.setPosition(servoPosHold);
        }
    }

}
