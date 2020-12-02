package com.acmerobotics.util;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robot.RoadrunnerDrive;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 537.6; //update this later with the omni wheel encoder ticks per rev
    public static double WHEEL_RADIUS = 0.5; // inches (radius of the omni wheel) fill in later cause idk
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    //need to measure some things for this
    public static double PARALLEL_X = 0; // X is the up and down direction
    public static double PARALLEL_Y = 0; // Y is the strafe direction

    public static double PERPENDICULAR_X = 0;
    public static double PERPENDICULAR_Y = 0;

    private DcMotorEx parallelEncoder, perpendicularEncoder;

    private RoadrunnerDrive drive;

    public TwoWheelTrackingLocalizer(Robot robot){
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        parallelEncoder = robot.getMotor("parallelEncoder");
        perpendicularEncoder = robot.getMotor("perpendicularEncoder");

        //TODO reverse encoder dirctions (don't think I'll have to do this, but I might)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading(){
        return drive.getRawExternalHeading();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions(){
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()),
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition())
        );
    }

    public List<Double> getWheelVelocities(){
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getVelocity()),
                encoderTicksToInches(perpendicularEncoder.getVelocity()) //this might be a problem later
                // (peek at roadrunner quickstart) but uhhhh we'll cross that bridge when we come to it.
        );
    }







}
