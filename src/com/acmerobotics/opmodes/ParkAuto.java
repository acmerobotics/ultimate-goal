package com.acmerobotics.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ParkAuto extends LinearOpMode {

    @Override
    public void runOpMode(){

        // drive instanciated

        waitForStart();

        while (!isStopRequested()){

            // move 80 in - (robot length / 2)

            // might strafe to the left/right to get rid of other robot's autos
        }

    }
}
