package com.acmerobotics.opmodes;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.robomatic.config.ConfigurationLoader;
import com.acmerobotics.robomatic.config.OpModeConfigurationActivity;
import com.acmerobotics.robomatic.demo.DemoConfig;
import com.acmerobotics.robot.TestConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.logging.Logger;

@Autonomous(name = "configOpMode")
public class ConfigOpMode extends LinearOpMode {

    @Override
    public void runOpMode(){

        TestConfig config = (TestConfig) new ConfigurationLoader(hardwareMap.appContext).getConfig();

        telemetry.addData("color", config.color);
        telemetry.addData("delay", config.delay);
        telemetry.addData("latched", config.latched);

        telemetry.update();


        waitForStart();

//        while(!isStopRequested()){
//
//        }

    }

}
