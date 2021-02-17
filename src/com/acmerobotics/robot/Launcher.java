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

    public DcMotor launcherShooterMotor;
    public Servo launcherServo;

    public double servoPosition = 0;
    public static double servoIncrement = 0.1;

    public static double lowShot = 0;
    public static double midShot = 0;
    public static double highShot = 0;


    public Launcher(Robot robot) {
        super("Launcher");

        launcherShooterMotor = robot.getMotor("shooterMotor");
        launcherShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        launcherServo = robot.getServo("shooterServo");
    }

    @Override
    public void update(Canvas Overlay){

    }


    public void shoot(){
        shooting = !shooting;

        if (shooting){
            launcherShooterMotor.setPower(1);
        }
        else{
            launcherShooterMotor.setPower(0);
        }
    }

    public void adjustAimUp(){
        servoPosition += servoIncrement;

        launcherServo.setPosition(servoPosition);
    }

    public void adjustAimDown(){
        servoPosition -= servoIncrement;

        launcherServo.setPosition(servoPosition);
    }

    public void shootLow(){
        launcherServo.setPosition(lowShot);
    }

    public void shootMid(){
        launcherServo.setPosition(midShot);
    }

    public void shootHigh(){
        launcherServo.setPosition(highShot);
    }
}
