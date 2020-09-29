package com.acmerobotics.robot;

import com.acmerobotics.robomatic.config.IntegerConfiguration;
import com.acmerobotics.robomatic.config.OpmodeConfiguration;

@OpmodeConfiguration
public class TestConfig {

    public enum AllianceColor {
        RED,
        BLUE,
    }

    public AllianceColor color;

    public enum StartLocation {
        CRATER,
        DEPOT
    }

    public StartLocation startLocation;


    public boolean latched;

    public boolean sampleBoth;

    public boolean playMusic;


    @IntegerConfiguration(max=10)
    public int delay;
}
