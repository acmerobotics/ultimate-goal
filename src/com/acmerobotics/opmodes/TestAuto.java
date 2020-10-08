package com.acmerobotics.opmodes;

import com.acmerobotics.robomatic.config.ConfigurationLoader;
import com.acmerobotics.util.TestConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "testAuto")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode(){
        TestConfig config = (TestConfig) new ConfigurationLoader(hardwareMap.appContext).getConfig();

        telemetry.addData("color", config.color);
        telemetry.addData("location", config.startLocation);
        telemetry.update();

        waitForStart();

    }
}
