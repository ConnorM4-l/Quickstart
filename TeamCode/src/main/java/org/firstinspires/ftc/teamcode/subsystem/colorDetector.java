package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//import org.firstinspires.ftc.teamcode.ColorDetector;

public class colorDetector {
    private ColorSensor colorSee = null;

    public colorDetector(HardwareMap hardwareMap) {
        colorSee = hardwareMap.get(ColorSensor.class, "colorDetector");
    }

    public void update() {
        int isGreen = colorSee.green();
    }
}
