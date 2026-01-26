package org.firstinspires.ftc.teamcode.teleop;


import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.InShotZone;
import org.firstinspires.ftc.teamcode.util.RobotContext;

/*
Make the driver be able to change heading by a little bit
*/
@TeleOp
public class V2TeleOpBlue extends OpMode {
    private Follower follower;
    private PIDFController controller;
    //public static Pose startingPose;
    private Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;

    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private DcMotorSimple intake = null;

    private Outtake shotController;
    private Intake intakeController;
    private RobotContext robotContext;
    private InShotZone inShotZone;

    private double offsetHeading = 0;
    private Pose targetPose = new Pose(12.844, 134.239, 0);
    private double targetHeading = 0; //should be in radians
    private double offsetShotHeading = 0;

    private boolean following = false;
    private boolean intaking = false;

    private boolean shootingLRR = false;
    private boolean shootingRLR = false;
    private boolean shootingRRL = false;

    private double headingError = 0;
    private double headingGoal = 0;

    private boolean manualPick;

    private double targetVelocity = 900;

    boolean headingLock = true;

    private boolean manualShot = true;
    private boolean manualDrive = false;
    private boolean prevManualShot = false;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        robotContext = new RobotContext();
        inShotZone = new InShotZone();

        controller = new PIDFController(new PIDFCoefficients(1.5, 0.0, 0.05, 0.033));
        startingPose = robotContext.getStartingPose();

        follower.setStartingPose(startingPose);
        follower.update();

        controller.setCoefficients(new PIDFCoefficients(1.5, 0.0, 0.05, 0.033));

        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        intake = hardwareMap.get(DcMotorSimple.class, "intakeMotor");

        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);

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
        //shotController.update(targetVelocity);
        setHeadingGoal();
        //shotController.update(distanceFromGoal());

        controller.updateError(getHeadingError());


        if (gamepad1.a) {
            intakeController.spin(1);
        } else if (gamepad1.b) {
            intakeController.spin(-1);
        } else {
            intakeController.spin(0);
        }

        if (!manualDrive) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, controller.run(), true);
        } else {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }

        if (manualShot) {
            //inShotZone.update(follower.getPose());
            //inShotZone.isInShotZone() && getHeadingError() < 0.1
            if ((Math.abs(getHeadingError()) < 0.1) || gamepad1.dpad_left) {
                //gamepad1.rumble(100);
                if (gamepad1.right_trigger > 0.5) {
                    shotController.shootBoth();
                } else if (gamepad1.left_bumper) {
                    shotController.shootLeft();
                } else if (gamepad1.right_bumper) {
                    shotController.shootRight();
                } else {
                    shotController.noShoot();
                }
            }
        } else {
            //do the automatic shooting process with the sorting from color sensors and shotOrdered from the shotControllers
        }

        if (gamepad2.right_trigger > 0.5) {
            gamepad2.rumble(100);
            if (gamepad2.dpad_up) {
                targetVelocity = 1400;
            } else if (gamepad2.dpad_down) {
                targetVelocity = 1600;
            } else if (gamepad2.dpad_left) {
                targetVelocity = 1240;
            } else if (gamepad2.aWasPressed()) {
                targetVelocity = 1100;
            }

            if (gamepad2.leftBumperWasPressed()) {
                targetVelocity -= 10;
            } else if (gamepad2.rightBumperWasPressed()) {
                targetVelocity += 10;
            }
        }

        if (gamepad1.xWasPressed()) {
            manualShot = true;
        } else if (gamepad1.yWasPressed()) {
            manualShot = false;
        }


        if (gamepad1.left_trigger > 0.5 && prevManualShot) {
            prevManualShot = false;
            manualDrive = !manualDrive;
        } else if (gamepad1.left_trigger > 0.5){
            prevManualShot = true;
        }

        if (gamepad1.dpad_down) {
            manualPick = true;
        } else if (gamepad1.dpad_up) {
            manualPick = false;
        }


        telemetry.addData("target velocity", targetVelocity);
        telemetry.addData("offsetShotHeading", offsetShotHeading);
        telemetry.addData("heading goal", headingGoal);
        telemetry.addData("is aligned", isAligned());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("heading error", headingError);
        telemetry.addData("is aligned", isAligned());
        //telemetry.addData("In shotZone", inShotZone.isInShotZone());
        telemetry.addData("distanceFromGoal", distanceFromGoal());
        telemetry.addData("manual shot", manualDrive);
        telemetry.addData("manual pick", manualPick);
        telemetry.update();
    }

    public void setHeadingGoal() {
        headingGoal = Math.atan2(
                targetPose.getY() - follower.getPose().getY(),
                targetPose.getX() - follower.getPose().getX());
    }
    public double distanceFromGoal() {
        return Math.sqrt(Math.pow(targetPose.getX() - follower.getPose().getX(), 2) + Math.pow(targetPose.getY() - follower.getPose().getY(), 2));
    }
    public boolean isAligned() {
        return Math.abs(follower.getHeading() - targetHeading) < 0.1;
    }

    public double getHeadingError() {
        headingError = MathFunctions.getTurnDirection(follower.getPose().getHeading(), headingGoal) * MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), headingGoal);
        return headingError;
    }
}

