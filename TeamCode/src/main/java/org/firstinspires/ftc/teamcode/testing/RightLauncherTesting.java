package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "RightLauncherTesting")
public class RightLauncherTesting extends OpMode {
    private DcMotorEx rightLauncher = null;
    @Override
    public void init() {
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
    }

    @Override
    public void loop() {
        rightLauncher.setPower(0.5);
    }

}
