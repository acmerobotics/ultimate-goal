package com.acmerobotics.opmodes.auto;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Drive Auto")
public class DriveAutoTest extends LinearOpMode {

    private int state;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, true, telemetry);
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
            telemetry.addData("YTarget", drive.Ytarget);
            telemetry.addData("error", drive.error);
            telemetry.addData("correction", drive.correction);
            telemetry.addData("motor 0 pos", drive.getMotorZeroCurrentPos());
            telemetry.addData("current power", drive.motors[0].getPower());
            telemetry.update();

            drive.motors[0].setPower(1);
            drive.motors[1].setPower(1);
            drive.motors[2].setPower(1);
            drive.motors[3].setPower(1);



                    /*
                   if(drive.atYPosition()){
                        drive.stopMotors();
                        drive.resetEncoders();
                        state++;
                    }




                     */




        }

    }
}
