package com.acmerobotics.opmodes;

import com.acmerobotics.robomatic.config.ConfigurationLoader;
import com.acmerobotics.robot.ACMERobot;
import com.acmerobotics.util.TestConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "config_test")
public class ConfigOpMode extends LinearOpMode {

    @Override
    public void runOpMode(){

        TestConfig config = (TestConfig) new ConfigurationLoader(hardwareMap.appContext).getConfig();

        ACMERobot robot = new ACMERobot(this);

        telemetry.addData("color", config.color);
        telemetry.addData("start location", config.startLocation);
        telemetry.update();

        waitForStart();

        while (!isStopRequested()){
            if (config.color == TestConfig.AllianceColor.RED){
                robot.drive.moveForward(12);
                telemetry.addLine("move forward");
            }

            if (config.color == TestConfig.AllianceColor.BLUE){
                robot.drive.turnRight(180);
                telemetry.addLine("turn right");
            }

            telemetry.update();
            robot.update();
        }
    }
}
