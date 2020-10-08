package com.acmerobotics.opmodes.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.ACMERobot;
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
        ACMERobot robot = new ACMERobot(this);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry Telemetry = dashboard.getTelemetry();

        waitForStart();


        while(!isStopRequested()) {

//            Telemetry.addData("powr 0", robot.drive.motors[0].getPower());
//            Telemetry.addData("powr 1", robot.drive.motors[1].getPower());
//            Telemetry.addData("powr 2", robot.drive.motors[2].getPower());
//            Telemetry.addData("powr 3", robot.drive.motors[3].getPower());
//
//            Telemetry.addLine();
//
//            Telemetry.addData("target", robot.drive.target);
//
//            Telemetry.addData("currentPos0", robot.drive.motors[0].getCurrentPosition());
//            Telemetry.addData("currentPos1", robot.drive.motors[1].getCurrentPosition());
//            Telemetry.addData("currentPos2", robot.drive.motors[2].getCurrentPosition());
//            Telemetry.addData("currentPos3", robot.drive.motors[3].getCurrentPosition());
//
//            Telemetry.addLine();
//
//            Telemetry.addData("error0", robot.drive.error0);
//            Telemetry.addData("error1", robot.drive.error1);
//            Telemetry.addData("error2", robot.drive.error2);
//            Telemetry.addData("error3", robot.drive.error3);
//
//            Telemetry.addLine();
//
//            Telemetry.addData("corrrection0", robot.drive.correction0);
//            Telemetry.addData("corrrection1", robot.drive.correction1);
//            Telemetry.addData("corrrection2", robot.drive.correction2);
//            Telemetry.addData("corrrection3", robot.drive.correction3);
//
//            Telemetry.addData("at Position", robot.drive.atTurningPosition());
//            Telemetry.addData("angle", robot.drive.getAngle());
//

            //Telemetry.addData("heading (radians)",robot.drive.imuSensor.getValue());
            //telemetryData.addData("heading (degrees)", getAngle());
            //Telemetry.addData("heading imu (degrees)", Math.toDegrees((double) robot.drive.imuSensor.getValue()));
            //Telemetry.addData("inTeleOP", robot.drive.inTeleOp);

            //Telemetry.update();
//
            if (gamepad1.y) {
                robot.drive.moveForward(distance);
            }
//
            if (gamepad1.x){
                robot.drive.turnRight(90);
            }
//
            if (gamepad1.a) {
                robot.drive.motors[0].setPower(0.5);
                robot.drive.motors[1].setPower(0.5);
                robot.drive.motors[2].setPower(0.5);
                robot.drive.motors[3].setPower(0.5);
            }

            if (gamepad1.b) {
                robot.drive.motors[0].setPower(0);
                robot.drive.motors[1].setPower(0);
                robot.drive.motors[2].setPower(0);
                robot.drive.motors[3].setPower(0);
            }

            robot.update();

        }

    }
}
