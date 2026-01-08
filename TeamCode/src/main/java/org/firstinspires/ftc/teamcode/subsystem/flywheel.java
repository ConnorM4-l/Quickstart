package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.ftc.localization.Encoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.VelocityPIDController;

public class flywheel {
    ElapsedTime accTimer = new ElapsedTime();

    double accTime = 0;

    private double power = 0;

    private final VelocityPIDController launcherController = new VelocityPIDController();

    private DcMotorEx leftLauncher;
    private DcMotorEx rightLauncher;

    private Encoder rightLauncherEncoder;

    public flywheel(HardwareMap hardwareMap) {
        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");

        rightLauncherEncoder = new Encoder(rightLauncher);



        accTimer.reset();
    }

    public void update(double LAUNCHER_TARGET_VELOCITY) {
//        power = launcherController.update(LAUNCHER_TARGET_VELOCITY, rightLauncherEncoder.getDeltaPosition() * 60.0 / 28);
        power = launcherController.update(LAUNCHER_TARGET_VELOCITY, -rightLauncher.getVelocity());

        setPower(power);
    }

    private void setPower(double pow) {
        leftLauncher.setPower(pow);
        rightLauncher.setPower(pow);
    }

    public double getErr() {
        return launcherController.getErr();
    }

    public double getVelocity() {
        return launcherController.getVelocity();
    }

    public double getAcceleration() {
        return launcherController.getAcceleration();
    }
}
