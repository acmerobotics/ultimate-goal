package com.acmerobotics.robot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.acmerobotics.util.RPMTool;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Config
public class Launcher extends Subsystem {

    private boolean shooting = false;

    public DcMotor launcherShooterMotorFront;
    public DcMotor launcherShooterMotorBack;
    public Servo aimServo; //the higher position the lower the aim will be
    public Servo launcherServo;

    public double servoPosition = 0;
    public static double servoIncrement = 0.05;

    public static double lowShot = 0.6;
    public static double midShot = 0.5;
    public static double highShot = 0.25;

    public static double kickPosition = 1;
    public static double resetPosition = 0.75;


    public Launcher(Robot robot) {
        super("Launcher");

        launcherShooterMotorFront = robot.getMotor("shooterMotorFront");
        launcherShooterMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherShooterMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherShooterMotorBack = robot.getMotor("shooterMotorBack");
        launcherShooterMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherShooterMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

        aimServo = robot.getServo("aimServo");
        launcherServo = robot.getServo("shooterServo");
    }

    @Override
    public void update(Canvas Overlay){
        telemetryData.addData("aimPosition", aimServo.getPosition());
        telemetryData.addData("kickerPosition", launcherServo.getPosition());
        telemetryData.addData("shooting", shooting);
    }


    public void shoot(){
        shooting = !shooting;

        if (shooting){
            launcherShooterMotorFront.setPower(1);
            launcherShooterMotorBack.setPower(1);
        }
        else{
            launcherShooterMotorFront.setPower(0);
            launcherShooterMotorBack.setPower(0);
        }
    }


    public void kickRing(){
        launcherServo.setPosition(kickPosition);
    }


    public void resetKicker(){
        launcherServo.setPosition(resetPosition);
    }


    public void adjustAimUp(){
        servoPosition -= servoIncrement;

        aimServo.setPosition(servoPosition);
    }

    public void adjustAimDown(){
        servoPosition += servoIncrement;

        aimServo.setPosition(servoPosition);
    }

    public void setTowerLevel(int level){
        if (level == 1){
            shootLow();
        }

        if (level == 2){
            shootMid();
        }

        if (level == 3){
            shootHigh();
        }
    }

    private void shootLow(){
        aimServo.setPosition(lowShot);
    }

    public void shootMid(){
        aimServo.setPosition(midShot);
    }

    public void shootHigh(){
        aimServo.setPosition(highShot);
    }
}
