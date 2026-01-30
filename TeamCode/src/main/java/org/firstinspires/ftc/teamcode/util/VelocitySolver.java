package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class VelocitySolver {
    public static double A = 99.6517;
    public static double B = 479.46875;
    public double getVelocity(double distance) {
        return A * Math.sqrt(distance) + B;
    }
}