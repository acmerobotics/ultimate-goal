package com.acmerobotics.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.robomatic.util.StickyGamepad;
import com.acmerobotics.robot.TestRobot;
import com.acmerobotics.robot.intake2;
import com.acmerobotics.robot.intake3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "testTeleOp")
public class TestTele extends LinearOpMode {

    @Override
    public void runOpMode(){

        //DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class,  "motor1");
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        intake2 intake2 = new intake2(hardwareMap);
        intake3 intake3 = new intake3(hardwareMap);
        TestRobot testRobot = new TestRobot(gamepad1, gamepad2);

        testRobot.teleOpActions.add(intake2);
        testRobot.teleOpActions.add(intake3);

        waitForStart();

        while (!isStopRequested()) {

            testRobot.updateTeleOpAction();

            dashboardTelemetry.addData("power", intake2.motor1.getPower());
            dashboardTelemetry.addData("a", gamepad1.a);
            dashboardTelemetry.addData("b", gamepad1.b);

            telemetry.update();
            dashboardTelemetry.update();
            stickyGamepad.update();
        }

    }
}
