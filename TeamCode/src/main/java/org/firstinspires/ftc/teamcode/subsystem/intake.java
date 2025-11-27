package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake {
    private DcMotorSimple intakeMotor = null;

    public intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(CRServo.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void update(double powerRequested) {
        intakeMotor.setPower(powerRequested);
    }
}
