package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.colorDetector;
import org.firstinspires.ftc.teamcode.util.InShotZone;
import org.firstinspires.ftc.teamcode.util.RobotContext;

@TeleOp
public class V2TeleOpBlue extends OpMode {

    // -------------------- Core Drive --------------------
    private Follower follower;
    private PIDFController controller;
    private Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;

    private double offsetHeading = 0;
    private Pose targetPose = new Pose(12.844, 134.239, 0);
    private double targetHeading = 0; //should be in radians
    private double offsetShotHeading = 0;

    private double headingError = 0;
    private double headingGoal = 0;

    boolean headingLock = true;

    private boolean following = false;
    private boolean intaking = false;

    // -------------------- Hardware --------------------
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private DcMotorSimple intake = null;

    // -------------------- Subsystems / Utils --------------------
    private Outtake shotController;
    private Intake intakeController;
    private RobotContext robotContext;
    private InShotZone inShotZone;

    private colorDetector ballColors;

    // -------------------- Shooting State --------------------
    private boolean autoShotActive = false;

    private boolean shootingLRR = false;
    private boolean shootingRLR = false;
    private boolean shootingRRL = false;

    private double targetVelocity = 900;

    private boolean manualShot = true;
    private boolean manualDrive = false;
    private boolean prevManualShot = false;

    private boolean manualPick;

    // Driver 2 selected order (Limelight motif): 21=GPP, 22=PGP, 23=PPG
    private int motif = 23;          // default to PPG unless driver changes it
    private int positionGreen = 0;   // 1 left, 2 right, 3 back, 0 unknown

    // -------------------- Timers --------------------
    private Timer teleOpTimer1;
    private double teleOpTime;

    // -------------------- Trigger Edge / Debounce --------------------
    private boolean rtHeld = false;      // debounced "pressed"
    private boolean rtPrevHeld = false;  // previous loop

    private boolean ltHeld = false;
    private double ltPressStartTime = 0.0;

    // tune these
    private static final double LT_PRESS_TH = 0.55;   // press threshold
    private static final double LT_RELEASE_TH = 0.45; // release threshold
    private static final double LT_HOLD_SEC = 0.30;   // tap vs hold boundary


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

        ballColors = new colorDetector(hardwareMap);

        teleOpTimer1 = new Timer();
    }

    @Override
    public void start() {
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

        teleOpTime = teleOpTimer1.getElapsedTimeSeconds();


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


// -------------------- AUTO SHOOT MODE --------------------
            if (!autoShotActive && rightTriggerPressedEvent()) {
                // run ONCE at the start
                ballColors.update();
                positionGreen = ballColors.getGreenPosition();

                shotController.setMotif(motif);
                shotController.setPositionGreen(positionGreen);

                autoShotActive = true;
            }

// keep advancing the outtake state machine until it finishes
            if (autoShotActive) {
                shotController.shootOrdered();
                if (!shotController.isStillShooting()) {
                    autoShotActive = false;
                    shotController.noShoot();
                }
            }
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

        // ----- LEFT TRIGGER: tap = noShoot, hold = reverseShoot -----
        double lt = gamepad1.left_trigger;

// hysteresis to prevent flicker
        if (!ltHeld && lt > LT_PRESS_TH) {
            ltHeld = true;
            ltPressStartTime = teleOpTime;   // record start time
        } else if (ltHeld && lt < LT_RELEASE_TH) {
            // released -> decide tap vs hold
            double heldSec = teleOpTime - ltPressStartTime;
            ltHeld = false;

            if (heldSec < LT_HOLD_SEC) {
                // TAP
                shotController.noShoot();
            }
            // if it was a hold, reverseShoot() was already being applied while held
        }

// while held long enough -> reverse shoot
        if (ltHeld && (teleOpTime - ltPressStartTime) >= LT_HOLD_SEC) {
            shotController.reverseShoot();
        }


        // -------------------- Driver 2 selects motif (order) --------------------
// Mapping:
// gp2.x -> 21 (GPP)
// gp2.y -> 22 (PGP)
// gp2.b -> 23 (PPG)
        if (gamepad2.xWasPressed()) {
            motif = 21;
            gamepad2.rumble(150);
        } else if (gamepad2.yWasPressed()) {
            motif = 22;
            gamepad2.rumble(150);
        } else if (gamepad2.bWasPressed()) {
            motif = 23;
            gamepad2.rumble(150);
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

    // Call this each loop
    public boolean rightTriggerPressedEvent() {
        // Hysteresis: press at 0.55, release at 0.45 (prevents flicker near 0.5)
        double rt = gamepad1.right_trigger;

        if (!rtHeld && rt > 0.55) rtHeld = true;
        else if (rtHeld && rt < 0.45) rtHeld = false;

        boolean pressedEvent = rtHeld && !rtPrevHeld;  // true only once per press
        rtPrevHeld = rtHeld;

        return pressedEvent;
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
