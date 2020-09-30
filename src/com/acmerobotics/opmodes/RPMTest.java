package com.acmerobotics.opmodes;

import com.acmerobotics.util.RPMTool;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RPMTest extends LinearOpMode {

    public void runOpMode(){

        RPMTool rpm = new RPMTool();

        waitForStart();

        while(!isStopRequested()){

        }

    }

}
