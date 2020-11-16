package com.acmerobotics.opmodes.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

public interface TeleOpActionImpl {
    void action(Gamepad gamepad1, Gamepad gamepad2);
}
