package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.TripleShot;

@Disabled
@TeleOp
public class V1TeleOp extends OpMode {

    private DcMotor bl = null;
    private DcMotor fl = null;
    private DcMotor br = null;
    private DcMotor fr = null;
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private Outtake shotController;
    private drivetrain movementController;
    private Intake intakeController;

    private boolean shotPressed = false;
    private double launcherVelocity = 1000;

    @Override
    public void init() {
        bl = hardwareMap.get(DcMotor.class, "bl");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shotController = new Outtake(hardwareMap);
        movementController = new drivetrain(hardwareMap);
        intakeController = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        shotController.update(launcherVelocity);
        movementController.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad2.left_bumper) {
            shotController.shootLeft();
        } else if (gamepad2.right_bumper) {
            shotController.shootRight();
        } else {
            shotController.noShoot();
        }

        if (gamepad2.xWasPressed()) {
            launcherVelocity += 100;
        }

        if (gamepad2.yWasPressed()) {
            launcherVelocity -= 100;
        }

        //telemetry.addData("Error", shotController.getErr());
        //telemetry.addData("Launch State", shotController.getLaunchingState());
        telemetry.addData("Target Velocity", launcherVelocity);
        telemetry.update();
    }
}