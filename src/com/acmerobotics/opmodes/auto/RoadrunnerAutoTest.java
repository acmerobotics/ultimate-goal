package com.acmerobotics.opmodes.auto;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robot.ACMERobot;
import com.acmerobotics.robot.RoadrunnerDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RoadrunnerAutoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ACMERobot robot = new ACMERobot(this);

        Pose2d startPose = new Pose2d(-70, 26, Math.toRadians(0));

        robot.drive.setCurrentEstimatedPose(startPose);

        Trajectory startToWobble = robot.drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-60, 26), Math.toRadians(0))
                .build();

        Trajectory wobbleToMidPoint = robot.drive.trajectoryBuilder(startToWobble.end())
                .splineTo(new Vector2d(-25, 26), Math.toRadians(0))
                .build();

        Trajectory midPointToPark = robot.drive.trajectoryBuilder(wobbleToMidPoint.end())
                .splineTo(new Vector2d(0, 35), Math.toRadians(0))
                .build();

        robot.drive.followTrajectory(startToWobble);
        //use the servo to grab the wobble goal mech
        robot.drive.followTrajectory(wobbleToMidPoint);
        robot.drive.followTrajectory(midPointToPark);



    }
}
