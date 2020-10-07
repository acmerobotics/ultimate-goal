package com.acmerobotics.opmodes.auto;

import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Drive Auto")
public class DriveAutoTest extends LinearOpMode {

    private int state;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, true);
        ElapsedTime time = new ElapsedTime();

        state = 0;

        drive.resetEncoders();
      //  drive.resetAngle();
        time.reset();

        telemetry.addData("state", state);
        telemetry.update();

        waitForStart();


        while(!isStopRequested()) {

            telemetry.addData("state", state);
            telemetry.update();

            switch (state) {

                case 0:

                    drive.moveForward(5);
                    state++;

                    break;

                case 1:

                    if(drive.atYPosition()){
                        drive.stopMotors();
                        drive.resetEncoders();
                        state++;
                    }

                    break;


            }

        }

    }
}
