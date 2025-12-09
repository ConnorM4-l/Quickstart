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
    public static Pose startingPose; //See ExampleAuto to understand how to use this
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
    private Pose targetPose = new Pose(-67, -67, 0);
    private double targetHeading = 0;

    private boolean following = false;
    private boolean intaking = false;

    private boolean shootingLRR = false;
    private boolean shootingRLR = false;
    private boolean shootingRRL = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(V2TeleOpBlue.startingPose == null ? new Pose() : V2TeleOpBlue.startingPose);
        follower.update();

        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        sGate = hardwareMap.get(Servo.class, "sGate");

        intake = hardwareMap.get(DcMotorSimple.class, "intakeMotor");

        intakeController = new Intake(hardwareMap);
        shotController = new Outtake(hardwareMap, 0.5, 0.5);
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

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true, // Robot Centric),
                offsetHeading);
        if (gamepad1.a) {
            intaking = true;
        } else if (gamepad1.b) {
            intaking = false;
        }
        if (intaking) {
            intakeController.spin(1);
        }
        if (gamepad1.dpad_right) {
            intakeController.gateRight();
        } else if (gamepad1.dpad_left) {
            intakeController.gateLeft();
        }
        if (gamepad1.xWasPressed()) {
            following = true;
        } else if (gamepad1.yWasPressed()) {
            following = false;
        }
        shotController.update(distanceFromGoal(), 1, 0.5);
        if (following) {
            follower.turnTo(desiredHeading());
            if (gamepad1.dpad_left) {
                shootingLRR = true;
            }
            if (shootingLRR) {
                while (shotController.isStillShooting()) {
                    shotController.LRRShoot(isAligned());
                }
            }
            if (shotController.isStillShooting()) {
                shotController.LRRShoot(isAligned());
            } else if (isAligned() && gamepad1.dpad_up){
                shotController.shootLeft();
            }
        }
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

