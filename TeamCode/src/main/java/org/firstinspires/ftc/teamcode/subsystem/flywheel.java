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

    private double rightPower = 0;
    private double leftPower = 0;

    private final VelocityPIDController rightLauncherController = new VelocityPIDController();
    private final VelocityPIDController leftLauncherController = new VelocityPIDController();


    private DcMotorEx leftLauncher;
    private DcMotorEx rightLauncher;

    private Encoder rightLauncherEncoder;
    private Encoder leftLauncherEncoder;

    public flywheel(HardwareMap hardwareMap) {
        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");

        rightLauncherEncoder = new Encoder(rightLauncher);
        leftLauncherEncoder = new Encoder(leftLauncher);


        accTimer.reset();
    }

    public void update(double LAUNCHER_TARGET_VELOCITY) {
//        power = launcherController.update(LAUNCHER_TARGET_VELOCITY, rightLauncherEncoder.getDeltaPosition() * 60.0 / 28);
        rightPower = rightLauncherController.update(LAUNCHER_TARGET_VELOCITY, -rightLauncher.getVelocity());
        leftPower = leftLauncherController.update(LAUNCHER_TARGET_VELOCITY, -leftLauncher.getVelocity());

        setPower();
    }

    private void setPower() {
        leftLauncher.setPower(leftPower);
        rightLauncher.setPower(rightPower);
    }

    public double getLeftErr() {
        return leftLauncherController.getErr();
    }

    public double getRightErr() {return rightLauncherController.getErr();}

    public double getLeftVelocity() {
        return leftLauncherController.getVelocity();
    }

    public double getRightVelocity() {
        return rightLauncherController.getVelocity();
    }

    public double getLeftAcceleration() {
        return leftLauncherController.getAcceleration();
    }

    public double getRightAcceleration() {
        return rightLauncherController.getAcceleration();
    }
}
