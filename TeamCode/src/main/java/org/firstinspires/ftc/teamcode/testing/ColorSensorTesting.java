package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.subsystem.colorDetector;

@TeleOp(name = "Color Sensor Testing")
public class ColorSensorTesting extends OpMode {
    private ColorSensor colorSensor;
    private colorDetector colorDetector;


    @Override
    public void init() {
        colorSensor = hardwareMap.get(ColorSensor.class, "leftColorSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "rightColorSensor");

        colorDetector = new colorDetector(hardwareMap);
    }

    @Override
    public void loop() {
        colorDetector.update();

        int left = colorDetector.detectLeftBallColor();
        int right = colorDetector.detectRightBallColor();

        telemetry.addData("Green Position", colorDetector.getGreenPosition());
        telemetry.addData("Left Color", left);
        telemetry.addData("Left Hue", colorDetector.getLeftHue());
        telemetry.addData("Left Saturation", colorDetector.getLeftSaturation());
        telemetry.addData("Left Value", colorDetector.getLeftValue());
        telemetry.addData("Right Color", right);
        telemetry.addData("Right Hue", colorDetector.getRightHue());
        telemetry.addData("Right Saturation", colorDetector.getRightSaturation());
        telemetry.addData("Right Value", colorDetector.getRightValue());
    }
}
