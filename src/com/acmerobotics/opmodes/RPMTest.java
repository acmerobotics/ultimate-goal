package com.acmerobotics.opmodes;

import com.acmerobotics.util.RPMTool;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "rpmTest")
public class RPMTest extends LinearOpMode {

    public void runOpMode(){

        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "motor1");

        RPMTool rpm = new RPMTool(motor1, 560);

        waitForStart();

        while(!isStopRequested()){

            if (gamepad1.a){
                motor1.setPower(0.2);
            }

            if (gamepad1.b){

                motor1.setPower(0.5);
            }

            if (gamepad1.dpad_up){
                rpm.setRPM(61);
            }

            if (gamepad1.x){
                motor1.setPower(0);
            }

            telemetry.addData(" rpm", rpm.getRPM());
            telemetry.update();

        }

    }

}
