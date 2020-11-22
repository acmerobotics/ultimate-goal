package com.acmerobotics.opmodes.auto;

import com.acmerobotics.robomatic.config.ConfigurationLoader;
import com.acmerobotics.robot.ACMERobot;
import com.acmerobotics.util.Configuration;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class Auto extends LinearOpMode {

    public Configuration.AllianceColor blue = Configuration.AllianceColor.BLUE;
    public Configuration.AllianceColor red = Configuration.AllianceColor.RED;
    public Configuration.StartLocation A = Configuration.StartLocation.A;
    public Configuration.StartLocation B = Configuration.StartLocation.B;

    public Configuration config;

    public enum TargetZone {
        A,
        B,
        C
    }
    public TargetZone targetZone; // make sure vision sets targetZone

    @Override
    public void runOpMode(){

        config = (Configuration) new ConfigurationLoader(hardwareMap.appContext).getConfig();

        waitForStart();

        if (config.color == Configuration.AllianceColor.BLUE){

            if (config.startLocation == Configuration.StartLocation.A){
                runBlueA();
            }

            else {
                runBlueB();
            }

        }

        else{

            if (config.startLocation == Configuration.StartLocation.B){
                runRedB();
            }

            else {
                runRedA();
            }

        }
    }


    abstract void runBlueA();

    abstract void runBlueB();

    abstract void runRedB();

    abstract void runRedA();

}
