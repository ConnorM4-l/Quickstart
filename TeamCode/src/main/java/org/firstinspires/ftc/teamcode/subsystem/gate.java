package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class gate {
    private Servo sGate = null;

    public gate(HardwareMap hardwareMap) {
        sGate = hardwareMap.get(Servo.class, "sGate");
    }

    public void update(boolean beLeft) {
        if (beLeft) {
            sGate.setPosition(0.2);
        } else {
            sGate.setPosition(0.7);
        }
    }
}
