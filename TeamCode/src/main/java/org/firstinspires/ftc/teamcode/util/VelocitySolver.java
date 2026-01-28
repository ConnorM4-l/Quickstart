package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class VelocitySolver {
    public double getVelocity(double distance) {
        final double A = 99.6517;
        final double B = 479.46875;

        return A * Math.sqrt(distance) + B;
    }
}
