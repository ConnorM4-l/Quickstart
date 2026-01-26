package org.firstinspires.ftc.teamcode.subsystem;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class colorDetector {
    private final NormalizedColorSensor colorSee;

    // Re-used buffer to avoid allocating every loop
    private final float[] hsv = new float[] {0f, 0f, 0f};

    public colorDetector(HardwareMap hardwareMap) {
        colorSee = hardwareMap.get(NormalizedColorSensor.class, "colorDetector");
    }

    /** Updates internal HSV buffer from the sensor. Call once per loop before reading/printing. */
    public void update() {
        NormalizedRGBA colors = colorSee.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsv);  // hsv[0]=Hue (0..360), hsv[1]=Sat (0..1), hsv[2]=Val (0..1)
    }

    public int detectBallColor() {
        float h = hsv[0]; // 0..360
        float s = hsv[1]; // 0..1
        float v = hsv[2]; // 0..1

        // 1) Reject "no ball / too dark / too gray"
        if (v < 0.12f || s < 0.25f) {
            return 0;
        }

        // 2) Hue bands (tune these!)
        // Green is typically ~90..150 in Android HSV
        if (h >= 90f && h <= 150f) {
            return 1;
        }

        // Purple is typically ~250..310 (sometimes closer to 270..300)
        if (h >= 250f && h <= 310f) {
            return 2;
        }

        return 0;
    }

    // --- Separate getters (HSV values) ---
    public float getHue() {
        return hsv[0];
    }

    public float getSaturation() {
        return hsv[1];
    }

    public float getValue() {
        return hsv[2];
    }
}
