package com.acmerobotics.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.robomatic.util.StickyGamepad;
import com.acmerobotics.robot.ACMERobot;
import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.vision.vuforiaSubsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "testTeleOp")
public class TestTele extends LinearOpMode {

    public static double aimPosition = 0;
    //public static double kickerPosition = 0;

    public static double leftPowerMod = 0.05;
    public static double rightPowerMod = 0.05;

    boolean inAutomatic = false;

    boolean shooting = false;

    boolean ready = true;

    double distanceFromLine = 0;

    double getDistanceFromSpot = 0;

    double farPosition = 0; // farPos is the distance needed before switching servo position to further one


    @Override
    public void runOpMode(){

        ACMERobot robot = new ACMERobot(this);
        StickyGamepad stickyGamepad = new StickyGamepad(gamepad1);

//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        waitForStart();

        while (!isStopRequested()) {

            if (!inAutomatic) {
                Pose2d v = new Pose2d(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
                robot.drive.setPower(v);
            }

            if (stickyGamepad.a){
                inAutomatic = true;

                robot.drive.autoTrajectory(-24);

                // determine shooting angle

                ready = false;

            }

            if (inAutomatic && robot.drive.atStrafePositionAutomatic() && !shooting){
                robot.drive.stopVel();

                shooting = true;
            }

            if (shooting){

                if (distanceFromLine > farPosition){
                    // set position to shoot far
                    //robot.launcher.shootMid();
                }
                else{
                    // set position to shoot close up
                    // robot.launcher.shootHigh();
                }

                robot.launcher.shootMid();

                robot.runUntil(robot.launcher::isMaxVelocity);

                robot.shootRingA();
                robot.shootRingA();
                robot.shootRingA();

                robot.launcher.shoot();

                shooting = false;

                robot.drive.switchWheelDirections(true);

                ready = true;

                inAutomatic = false;
            }

            if (gamepad1.x){
                robot.drive.stopVel();
            }

            telemetry.addData("target visible: ", false);

            telemetry.update();
            stickyGamepad.update();
            robot.update();

        }

    }
}
