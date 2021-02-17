package com.acmerobotics.robot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class LauncherV2 extends Subsystem {

    private boolean shooting = false;

    public DcMotor launcherShooterMotor;
    public Servo launcherServo;

    public double servoPosition = 0;
    public static double servoIncrement = 0.1;


    public LauncherV2(Robot robot) {
        super("LauncherV2");

        launcherShooterMotor = robot.getMotor("shooterMotor");
        launcherShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        launcherServo = robot.getServo("shooterServo");
    }

    @Override
    public void update(Canvas Overlay){

    }


    public void shoot(){

    }

    public void adjustAimUp(){
        servoPosition += servoIncrement;
    }

    public void adjustAimDown(){
        servoPosition -= servoIncrement;
    }

}
