package com.acmerobotics.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveContants {

    public static final double TICKS_PER_REV = 537.6;
    public static final double MAX_RPM = 312.5;

    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0, getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double WHEEL_RADIUS = 2; //inches
    public static double GEAR_RATIO = 1; //double check w/ hardware once they finish the drive
    public static double TRACK_WIDTH = 1; //inches, measure once the drivetrain is finished

    public static double kV = 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;

    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            30.0, 30.0, 0.0, Math.toRadians(180.0),
            Math.toRadians(180.0), 0.0
    );

    public static double encoderTicksToInches(double ticks){
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm){
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60;
    }

    public static double getMotorVelocityF(double ticksPerSecond){
        return 32767 / ticksPerSecond;
    }
}
