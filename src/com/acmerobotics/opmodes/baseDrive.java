package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.util.StickyGamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp
public class baseDrive extends LinearOpMode {

    public static double leftPowerMod = 0.05;
    public static double rightPowerMod = 0.05;

    @Override
    public void runOpMode() {

        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);

        DcMotorEx[] motors = new DcMotorEx[4];

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry Telemetry = dashboard.getTelemetry();

        for (int i=0; i<4;i++){
            motors[i] = hardwareMap.get(DcMotorEx.class, "m" + i);
        }

        motors[0].setDirection(DcMotorEx.Direction.REVERSE);
        motors[1].setDirection(DcMotorEx.Direction.REVERSE);
        motors[2].setDirection(DcMotorEx.Direction.FORWARD);
        motors[3].setDirection(DcMotorEx.Direction.FORWARD);

        for (int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }

        waitForStart();

        while (!isStopRequested()) {


            if (gamepad1.a) {

                motors[0].setPower(0.5 + leftPowerMod);
                motors[1].setPower(0.5 + leftPowerMod);
                motors[2].setPower(0.5 + leftPowerMod);
                motors[3].setPower(0.5 + leftPowerMod);

            }

            if (stickyGamepad.left_bumper) {
                leftPowerMod += 0.05;
            }
            if (stickyGamepad.right_bumper) {
                rightPowerMod += 0.05;
            }

            if (stickyGamepad.x) {
                motors[0].setPower(0);
                motors[1].setPower(0);
                motors[2].setPower(0);
                motors[3].setPower(0);

            }

            Telemetry.addData("m0 power", motors[0].getPower());
            Telemetry.addData("m1 power", motors[1].getPower());
            Telemetry.addData("m2 power", motors[2].getPower());
            Telemetry.addData("m3 power", motors[3].getPower());
            Telemetry.addData("leftMod", leftPowerMod);
            Telemetry.addData("rightMod", rightPowerMod);
            Telemetry.update();

            stickyGamepad.update();

        }
    }
}

