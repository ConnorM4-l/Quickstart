package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private intakeWheels intake ;

    public Intake(HardwareMap hardwareMap) {
        intake = new intakeWheels(hardwareMap);
    }

    public void spin(double power) {
        intake.update(power);
    }
}
