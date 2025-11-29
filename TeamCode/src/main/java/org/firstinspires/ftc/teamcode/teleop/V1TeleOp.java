package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystem.drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.TripleShot;

@TeleOp(name = "SecondTeleOp")
public class V1TeleOp extends OpMode {

    private DcMotor bl = null;
    private DcMotor fl = null;
    private DcMotor br = null;
    private DcMotor fr = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private TripleShot shotController;
    private drivetrain movementController;

    private boolean shotPressed = false;
    private double launcherVelocity = 2700;

    @Override
    public void init() {
        bl = hardwareMap.get(DcMotor.class, "bl");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shotController = new TripleShot(hardwareMap);
        movementController = new drivetrain(hardwareMap);
    }

    @Override
    public void loop() {
        shotController.update(gamepad1.right_bumper, gamepad1.left_bumper, launcherVelocity);
        movementController.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.dpad_up) {
            launcherVelocity += 10;
        }
        if (gamepad1.dpad_down) {
            launcherVelocity -= 10;
        }

        telemetry.addData("Error", shotController.getErr());
        telemetry.addData("Launch State", shotController.getLaunchingState());
        telemetry.addData("Target Velocity", launcherVelocity);
        telemetry.update();
    }
}