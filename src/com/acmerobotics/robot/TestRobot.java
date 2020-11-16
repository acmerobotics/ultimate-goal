package com.acmerobotics.robot;

import com.acmerobotics.opmodes.teleop.TeleOpActionImpl;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;

public class TestRobot{ // this code would go in the Robot class

    public Gamepad gamepad1;
    public Gamepad gamepad2;

    public TestRobot(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public List<TeleOpActionImpl> teleOpActions = new ArrayList<>();

    public void updateTeleOpAction(){
        for(TeleOpActionImpl teleOpAction: teleOpActions){
            teleOpAction.action(gamepad1, gamepad2);
        }
    }
}
