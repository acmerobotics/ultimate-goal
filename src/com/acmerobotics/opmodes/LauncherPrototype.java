package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
the freeFlywheelSpeed will be the speed of the flywheel spinning at full power without any obstructions.
when freeFlywheelSpeed drops more than this threshold, the program will detect a ring has been launched.
This will be used to test the time it takes from shooting to ready to shoot again.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Launcher Prototype")

@Config
public class LauncherPrototype extends LinearOpMode {

    private double flywheelGearReduction;

    public static double motorPower; //dashboard configurable
    public static double freeFlywheelSpeed; //dashboard configurable
    public static double flywheelLaunchThreshold; //dashboard configurable

    @Override
    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelementry = dashboard.getTelemetry();

        DcMotor launcherAccelMotor = hardwareMap.get(DcMotor.class,"accelMotor");
        DcMotor launcherShooterMotor = hardwareMap.get(DcMotor.class,"shooterMotor");
        launcherAccelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //PID controlled velocity
        launcherShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //PID controlled velocity

        waitForStart();

        while (!isStopRequested()) {

        }
    }



}
