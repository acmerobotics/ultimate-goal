package com.acmerobotics.robot;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TankDriveEX")
public class TankDriveEX extends OpMode {
    public double leftSpeed;
    public double rightSpeed;

    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor leftBackMotor;
    public DcMotor rightBackMotor;


    public void init(){
leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
    }

    public void loop(){

        leftSpeed = gamepad1.left_stick_y+gamepad1.left_stick_x;
        rightSpeed = gamepad1.left_stick_y-gamepad1.left_stick_x;

        leftMotor.setPower(leftSpeed);
        leftBackMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
        rightBackMotor.setPower(rightSpeed);
    }

}
