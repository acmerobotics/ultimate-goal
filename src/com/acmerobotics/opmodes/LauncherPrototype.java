package com.acmerobotics.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.util.RPMTool;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

/*
the freeFlywheelSpeed will be the speed of the flywheel spinning at full power without any obstructions.
when freeFlywheelSpeed drops more than this threshold, the program will detect a ring has been launched.
This will be used to test the time it takes from shooting to ready to shoot again.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Launcher Prototype")

@Config
public class LauncherPrototype extends LinearOpMode {

    private double motorGearReduction = 4.0;
    private double LauncherGearReduction =1.0;
    private double flywheelGearReduction = motorGearReduction * LauncherGearReduction;
    public double ticksPerRevolution = 7 * flywheelGearReduction;

    public static double motorPower = 1; //dashboard configurable
    //public static double motorTargetVelocityRPM = 0; //dashboard configurable
    public static double freeFlywheelSpeed = 0; //dashboard configurable
    public static double flywheelLaunchThreshold = 0; //dashboard configurable
    private int loopsAboveThreshold=0;

    public double currentAccelMotorVelocityRPM;
    public double currentShooterMotorVelocityRPM;
    public double averageMotorVelocityRPM;

    private String flywheelStatus;
    private ArrayList<Double> speedResetTimes = new ArrayList<>();

    @Override
    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelementry = dashboard.getTelemetry();

        ElapsedTime flywheelSpinTime = new ElapsedTime();
        flywheelStatus = "Speeding Up";

        DcMotor launcherAccelMotor = hardwareMap.get(DcMotor.class,"accelMotor");
        DcMotor launcherShooterMotor = hardwareMap.get(DcMotor.class,"shooterMotor");
        launcherAccelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //PID controlled velocity
        launcherShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //PID controlled velocity
        launcherAccelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        RPMTool accelMotorRPMTool = new RPMTool(launcherAccelMotor,ticksPerRevolution);
        RPMTool shooterMotorRPMTool = new RPMTool(launcherShooterMotor,ticksPerRevolution);

        waitForStart();

        while (!isStopRequested()) {
            //set motorPower to 1 in dashboard to test the max rmp of the launcher, then set motorPower to 0 and set rpms for velocity PID

            launcherAccelMotor.setPower(motorPower);
            launcherShooterMotor.setPower(motorPower);

            currentAccelMotorVelocityRPM = accelMotorRPMTool.getRPM();
            currentShooterMotorVelocityRPM = shooterMotorRPMTool.getRPM();
            averageMotorVelocityRPM = (currentAccelMotorVelocityRPM + currentShooterMotorVelocityRPM) /2;

            if(Math.abs(freeFlywheelSpeed-averageMotorVelocityRPM) >= flywheelLaunchThreshold || flywheelStatus.equals("Speeding Up")) {

                if(flywheelStatus.equals("Ready to Launch")){
                    flywheelSpinTime.reset();
                }

                 flywheelStatus = "Speeding Up";

                 if(Math.abs(freeFlywheelSpeed-averageMotorVelocityRPM) <= flywheelLaunchThreshold) {
                     loopsAboveThreshold++;
                     if(loopsAboveThreshold >= 5){
                         Log.i("Flywheel Reset", String.valueOf(flywheelSpinTime.milliseconds()));
                         flywheelStatus = "Ready to Launch";
                         loopsAboveThreshold = 0;
                         speedResetTimes.add(flywheelSpinTime.milliseconds());
                     }
                 }
            }

            dashboardTelementry.addData("currentAccelMotorSpeed: ", currentAccelMotorVelocityRPM);
            dashboardTelementry.addData("currentShooterMotorSpeed: ", currentShooterMotorVelocityRPM);
            dashboardTelementry.addData("Flywheel Status: ", flywheelStatus);
            dashboardTelementry.update();

            Log.i("Average Motor Speed", String.valueOf(averageMotorVelocityRPM));
        }
        //outputting Flywheel Reset Times
        double averageResetTime = speedResetTimes.get(0);
        for(int i=1; i < speedResetTimes.size(); i++) {
            averageResetTime = (averageResetTime + speedResetTimes.get(i)) / 2;
        }
        Log.i("Average Reset Speed", String.valueOf(averageResetTime));
    }



}
