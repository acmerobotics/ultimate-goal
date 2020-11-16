package com.acmerobotics.robot;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.opmodes.teleop.TeleOpActionImpl;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake2 implements TeleOpActionImpl {
    public DcMotorEx motor1;

    public intake2(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotorEx.class,  "motor1");
    }

    @Override
    public void action(Gamepad gamepad1, Gamepad gamepad2){
        if (gamepad1.a) motor1.setPower(0.5);
    }

}
