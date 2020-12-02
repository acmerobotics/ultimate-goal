package com.acmerobotics.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DcMotors2 {

    DcMotor motor1;
    DcMotor motor2;

    public DcMotors2(HardwareMap hardwareMap){
        motor1 = hardwareMap.get(DcMotor.class, deviceName: "motor 1");
        motor2 = hardwareMap.get(DcMotor.class, deviceName: "motor2");
    }

    public void setPower(){
        motor1.setPower(0.5);
        motor2.setPower(1);
    }
}
