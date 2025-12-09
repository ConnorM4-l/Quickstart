package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private intakeWheels intake ;
    private gate gate;

    public Intake(HardwareMap hardwareMap) {
        intake = new intakeWheels(hardwareMap);
        gate = new gate(hardwareMap);
    }

    public void spin(double power) {
        intake.update(power);
    }

    public void gateLeft() {
        gate.update(true);
    }

    public void gateRight() {
        gate.update(false);
    }
}
