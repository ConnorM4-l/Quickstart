package org.firstinspires.ftc.teamcode.subsystem;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class colorDetector {

    // Hardware names in the RC config
    private final NormalizedColorSensor leftColorSensor;
    private final NormalizedColorSensor rightColorSensor;

    // Re-used buffers (no alloc per loop)
    private final float[] leftHSV  = new float[] {0f, 0f, 0f};
    private final float[] rightHSV = new float[] {0f, 0f, 0f};

    // Return codes
    public static final int NONE = 0;
    public static final int GREEN = 1;
    public static final int PURPLE = 2;

    public colorDetector(HardwareMap hardwareMap) {
        leftColorSensor  = hardwareMap.get(NormalizedColorSensor.class, "leftColorSensor");
        rightColorSensor = hardwareMap.get(NormalizedColorSensor.class, "rightColorSensor");
    }

    /** Call once per loop to update HSV buffers for BOTH sensors. */
    public void update() {
        updateSensor(leftColorSensor, leftHSV);
        updateSensor(rightColorSensor, rightHSV);
    }

    private void updateSensor(NormalizedColorSensor sensor, float[] hsvOut) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvOut);
    }

    /** Returns NONE(0), GREEN(1), PURPLE(2) for the LEFT sensor. */
    public int detectLeftBallColor() {
        return detectFromHSV(leftHSV);
    }

    /** Returns NONE(0), GREEN(1), PURPLE(2) for the RIGHT sensor. */
    public int detectRightBallColor() {
        return detectFromHSV(rightHSV);
    }

    /** Shared classification logic. Tune thresholds here. */
    private int detectFromHSV(float[] hsv) {
        float h = hsv[0]; // 0..360
        float s = hsv[1]; // 0..1
        float v = hsv[2]; // 0..1

        // Reject "no ball / too dark / too gray"
        if (v < 0.12f || s < 0.25f) return NONE;

        // Green (~90..150)
        if (h >= 90f && h <= 150f) return GREEN;

        // Purple (~250..310)
        if (h >= 250f && h <= 310f) return PURPLE;

        return NONE;
    }

    // --- Left HSV getters ---
    public float getLeftHue() { return leftHSV[0]; }
    public float getLeftSaturation() { return leftHSV[1]; }
    public float getLeftValue() { return leftHSV[2]; }

    // --- Right HSV getters ---
    public float getRightHue() { return rightHSV[0]; }
    public float getRightSaturation() { return rightHSV[1]; }
    public float getRightValue() { return rightHSV[2]; }
}
