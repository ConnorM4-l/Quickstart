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

    // Return codes (colors)
    public static final int NONE = 0;
    public static final int GREEN = 1;
    public static final int PURPLE = 2;

    // Return codes (green position)
    // 1 = left, 2 = right, 3 = back, 0 = unknown/none
    public static final int GREEN_LEFT = 1;
    public static final int GREEN_RIGHT = 2;
    public static final int GREEN_BACK = 3;
    public static final int GREEN_UNKNOWN = 0;

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

    /**
     * Returns green position code:
     * 1 = left, 2 = right, 3 = back, 0 = unknown/none
     *
     * Logic:
     * - If left sensor sees GREEN -> green is left
     * - Else if right sensor sees GREEN -> green is right
     * - Else if neither sees GREEN but both see a ball (purple/green) -> assume green is back
     * - Else -> unknown (not enough info / no balls)
     */
    public int getGreenPosition() {
        int left = detectLeftBallColor();
        int right = detectRightBallColor();

        if (left == GREEN) return GREEN_LEFT;
        if (right == GREEN) return GREEN_RIGHT;

        boolean leftHasBall = (left != NONE);
        boolean rightHasBall = (right != NONE);

        if (leftHasBall && rightHasBall) {
            // neither front sensor sees green, but both see balls -> green likely in back
            return GREEN_BACK;
        }

        return GREEN_UNKNOWN;
    }

    /** Shared classification logic. Tune thresholds here. */
    private int detectFromHSV(float[] hsv) {

        //Green
        // H 155     160    180     120
        // S 0.8     0.75   1       1
        // V 0.0588  0.0314 0.0078  0.0039
        //Purple
        // H
        // S
        // V
        float h = hsv[0]; // 0..360
        float s = hsv[1]; // 0..1
        float v = hsv[2]; // 0..1

        // Reject "no ball / too dark / too gray"
        if (v < 0.003f || s < 0.25f) return NONE;

        // Green (~90..180)
        if (h >= 90f && h <= 180f) return GREEN;

        // Purple (~180..310)
        if (h > 180f && h <= 310f) return PURPLE;

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
