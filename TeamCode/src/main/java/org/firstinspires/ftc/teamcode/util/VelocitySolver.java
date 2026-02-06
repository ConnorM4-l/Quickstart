package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class VelocitySolver {
    public static double A = 96.6354;
    public static double B = 496.18863;
    public double getVelocity(double distance) {
        return A * Math.sqrt(distance) + B;
    }
}