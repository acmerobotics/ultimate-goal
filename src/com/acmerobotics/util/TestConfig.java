package com.acmerobotics.util;

import com.acmerobotics.robomatic.config.OpmodeConfiguration;

@OpmodeConfiguration
public class TestConfig {

    public enum AllianceColor {
        RED,
        BLUE,
    }

    public AllianceColor color;

    public enum StartLocation {
        A,
        B
    }

    public StartLocation startLocation;

}
