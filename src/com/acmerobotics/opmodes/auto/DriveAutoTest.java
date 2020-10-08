package com.acmerobotics.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous(name="Drive Auto")
public class DriveAutoTest extends LinearOpMode {

    private int state;

    public static double distance = 12.56;

    @Override
    public void runOpMode() {
        Drive drive = new Drive(hardwareMap, false);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry Telemetry = dashboard.getTelemetry();

        waitForStart();


        while(!isStopRequested()) {

            Telemetry.addData("powr 0", drive.motors[0].getPower());
            Telemetry.addData("powr 1", drive.motors[1].getPower());
            Telemetry.addData("powr 2", drive.motors[2].getPower());
            Telemetry.addData("powr 3", drive.motors[3].getPower());

            Telemetry.addLine();

            Telemetry.addData("target", drive.target);

            Telemetry.addData("currentPos0", drive.motors[0].getCurrentPosition());
            Telemetry.addData("currentPos1", drive.motors[1].getCurrentPosition());
            Telemetry.addData("currentPos2", drive.motors[2].getCurrentPosition());
            Telemetry.addData("currentPos3", drive.motors[3].getCurrentPosition());

            Telemetry.addLine();

            Telemetry.addData("error0", drive.error0);
            Telemetry.addData("error1", drive.error1);
            Telemetry.addData("error2", drive.error2);
            Telemetry.addData("error3", drive.error3);

            Telemetry.addLine();

            Telemetry.addData("corrrection0", drive.correction0);
            Telemetry.addData("corrrection1", drive.correction1);
            Telemetry.addData("corrrection2", drive.correction2);
            Telemetry.addData("corrrection3", drive.correction3);

            Telemetry.addData("at Position", drive.atTurningPosition());
            Telemetry.addData("angle", drive.getAngle());

            Telemetry.update();

            if (gamepad1.y) {
                drive.turnLeft(90);
            }

            if (gamepad1.x){
                drive.turnRight(90);
            }

            if (gamepad1.a) {
                drive.motors[0].setPower(0.5);
                drive.motors[1].setPower(0.5);
                drive.motors[2].setPower(0.5);
                drive.motors[3].setPower(0.5);
            }

            if (gamepad1.b) {
                drive.motors[0].setPower(0);
                drive.motors[1].setPower(0);
                drive.motors[2].setPower(0);
                drive.motors[3].setPower(0);
            }

                drive.update();

        }

    }
}
