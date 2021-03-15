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

    public static double power = 0.825; // 0.975 for high, 0.825 for mid, x for low

    private boolean shooting = false;

    public DcMotor launcherShooterMotorFront;
    public DcMotor launcherShooterMotorBack;
    public Servo aimServo; //the higher position the lower the aim will be
    public Servo launcherServo;

    public static double servoAim = 0.71; // higher for lower shot position
    public static double servoAimCloseUp = 0.67;

    public static double kickPosition = 1;
    public static double resetPosition = 0.75;

    private static double TICKS_PER_REV = 25;
    private static double RPM = 5000;

    RPMTool rpmTool;


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

        rpmTool = new RPMTool(launcherShooterMotorBack, TICKS_PER_REV);
    }

    @Override
    public void update(Canvas Overlay){
        telemetryData.addData("aimPosition", aimServo.getPosition());
        telemetryData.addData("kickerPosition", launcherServo.getPosition());
        telemetryData.addData("shoot power", launcherShooterMotorFront.getPower());
        telemetryData.addData("velocity back motor", rpmTool.getRPM());
    }


    public void shoot(){
        shooting = !shooting;

        if (shooting){
            launcherShooterMotorFront.setPower(power);
            launcherShooterMotorBack.setPower(power);
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
        shooting = true;
        power += 0.05;

        launcherShooterMotorFront.setPower(power);
        launcherShooterMotorBack.setPower(power);
    }

    public void adjustAimDown(){
        shooting = true;
        power -= 0.05;

        launcherShooterMotorFront.setPower(power);
        launcherShooterMotorBack.setPower(power);
    }


    public void shootLow(){
        shooting = true;
        aimServo.setPosition(servoAim);
        power = 0.55;

        launcherShooterMotorFront.setPower(power);
        launcherShooterMotorBack.setPower(power);
    }

    public void shootMid(){
        shooting = true;
        aimServo.setPosition(servoAim);
        power = 0.825;

        launcherShooterMotorFront.setPower(power);
        launcherShooterMotorBack.setPower(power);
    }

    public void shootHigh(){
        shooting = true;
        aimServo.setPosition(servoAimCloseUp);
        power = 0.825;

        launcherShooterMotorFront.setPower(power);
        launcherShooterMotorBack.setPower(power);
    }

    public boolean isMaxVelocity() {

        return (rpmTool.getRPM() >= (RPM - 750)); //was 500 originally, trying to increase margin of error

    }
}
