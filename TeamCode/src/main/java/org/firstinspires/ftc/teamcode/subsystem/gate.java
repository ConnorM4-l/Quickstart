package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class gate {
    private Servo sGate = null;

    public static double leftPosition = 0.37;
    public static double rightPosition = 0.66;


    public gate(HardwareMap hardwareMap) {
        sGate = hardwareMap.get(Servo.class, "sGate");
    }

    public void update(boolean beLeft) {
        if (beLeft) {
            sGate.setPosition(leftPosition);
        } else {
            sGate.setPosition(rightPosition);
        }
    }
}
