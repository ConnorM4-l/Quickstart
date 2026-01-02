package org.firstinspires.ftc.teamcode.teleop;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

/*
Make the driver be able to change heading by a little bit
*/
@TeleOp
public class V2TeleOpBlue extends OpMode {
    private Follower follower;
    private Pose startingPose = new Pose(8, 8, 0); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;

    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private Servo sGate = null;
    private DcMotorSimple intake = null;

    private Outtake shotController;
    private Intake intakeController;

    private double offsetHeading = 0;
    private Pose targetPose = new Pose(5, 67, 0);
    private double targetHeading = 0;
    private double offsetShotHeading = 0;

    private boolean following = false;
    private boolean intaking = false;

    private boolean shootingLRR = false;
    private boolean shootingRLR = false;
    private boolean shootingRRL = false;

    private double targetVelocity = 1500;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        sGate = hardwareMap.get(Servo.class, "sGate");

        intake = hardwareMap.get(DcMotorSimple.class, "intakeMotor");

        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeController = new Intake(hardwareMap);
        shotController = new Outtake(hardwareMap);
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        shotController.update(targetVelocity);
        //shotController.update(distanceFromGoal());

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false); // Robot Centric)
        if (gamepad1.a) {
            intakeController.spin(1);
        } else if (gamepad1.b) {
            intakeController.spin(-1);
        } else{
            intakeController.spin(0);
        }

        if (gamepad2.dpad_right) {
            intakeController.gateRight();
        } else if (gamepad2.dpad_left) {
            intakeController.gateLeft();
        }

        if (gamepad1.x) {
            follower.turnTo(Math.toRadians(135));
        } else if (gamepad1.yWasPressed()) {
            //maybe somehow turn off that following
        }

        if (gamepad2.xWasPressed()) {
            targetVelocity = 1650;
        } else if (gamepad2.yWasPressed()) {
            targetVelocity = 1500;
        } else if (gamepad2.aWasPressed()) {
            targetVelocity = 1250;
        } else if (gamepad2.bWasPressed()) {
            targetVelocity = 1000;
        }

        if (gamepad2.left_bumper) {
            targetVelocity -= 10;
        } else if (gamepad2.right_bumper) {
            targetVelocity += 10;
        }

        if (gamepad1.left_bumper) {
            offsetShotHeading -= 1;
        } else if (gamepad1.right_bumper) {
            offsetShotHeading += 1;
        }

        if (gamepad1.right_bumper && gamepad1.left_bumper) {
            shotController.shootBoth();
        } else if (gamepad1.left_bumper) {
            shotController.shootLeft();
        } else if (gamepad1.right_bumper) {
            shotController.shootRight();
        } else {
            shotController.noShoot();
        }

        telemetry.addData("target velocity", targetVelocity);
        telemetry.addData("offsetShotHeading", offsetShotHeading);
        telemetry.update();
    }

    public double desiredHeading() {
        return Math.atan2(targetPose.getY() - follower.getPose().getY(), targetPose.getX()) - follower.getPose().getX();
    }
    public double distanceFromGoal() {
        return Math.sqrt(Math.pow(targetPose.getX() - follower.getPose().getX(), 2) + Math.pow(targetPose.getY() - follower.getPose().getY(), 2));
    }
    public boolean isAligned() {
        return Math.abs(follower.getHeading() - targetHeading) < 0.1;
    }
}

