package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class feeders {
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    public feeders(HardwareMap hardwareMap) {
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update(boolean left, boolean right) {
        if (left) {
            leftFeeder.setPower(1);
        } else {
            leftFeeder.setPower(0);
        }

        if (right) {
            rightFeeder.setPower(1);
        } else {
            rightFeeder.setPower(0);
        }
    }
}
