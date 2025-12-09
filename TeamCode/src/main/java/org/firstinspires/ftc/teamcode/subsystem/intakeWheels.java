package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intakeWheels {
    private DcMotorSimple intakeMotor = null;

    public intakeWheels(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorSimple.class, "intakeMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update(double powerRequested) {
        intakeMotor.setPower(powerRequested);
    }
}
