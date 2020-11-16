package com.acmerobotics.robot;

import com.acmerobotics.opmodes.teleop.TeleOpActionImpl;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake3 implements TeleOpActionImpl {
    DcMotorEx motor1;

    public intake3(HardwareMap hardwareMap){
        motor1 = hardwareMap.get(DcMotorEx.class,  "motor1");
    }

    @Override
    public void action(Gamepad gamepad1, Gamepad gamepad2){
        if (gamepad1.b) motor1.setPower(0.1);
    }
}
