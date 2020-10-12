package com.acmerobotics.util;

import com.acmerobotics.robomatic.config.OpmodeConfiguration;

@OpmodeConfiguration
public class Configuration {

    public enum AllianceColor{
        BLUE,
        RED
    }

    public AllianceColor color;


    public enum StartLocation{
        A,
        B
    }

    public StartLocation startLocation;

}
