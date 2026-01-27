package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.colorDetector;
import org.firstinspires.ftc.teamcode.util.RobotContext;

@TeleOp
public class V2TeleOpBlue extends OpMode {

    // -------------------- Core Drive --------------------
    private Follower follower;
    private PIDFController controller;
    private Pose startingPose;

    // Goal pose (used for heading lock in auto-aim)
    private Pose targetPose = new Pose(12.844, 134.239, 0);

    private double headingError = 0;
    private double headingGoal = 0;

    // -------------------- Subsystems --------------------
    private Outtake shotController;
    private Intake intakeController;
    private RobotContext robotContext;
    private colorDetector ballColors;

    // -------------------- Driver State --------------------
    // manualDrive=true  -> full manual rotation (right stick)
    // manualDrive=false -> auto heading lock to goal (PID turn)
    private boolean manualDrive = true;

    // Intake: A toggles forward on/off, B is hold reverse
    private boolean intakeToggledOn = false;
    private boolean aPrev = false;

    // Driver 2 selected motif (Limelight tag meaning): 21=GPP, 22=PGP, 23=PPG
    private int motif = 23;          // default motif
    private int positionGreen = 0;   // 1 left, 2 right, 3 back, 0 unknown

    // Shot modes:
    // QUICK   -> 3-ball burst, unordered (fast cycle)
    // ORDERED -> ordered routine based on motif + green position
    private enum ShotMode { QUICK, ORDERED }
    private ShotMode shotMode = ShotMode.QUICK; // default
    private boolean modeTogglePrev = false;      // edge detect for driver2 mode toggle

    // Driver 1 right-trigger two-press cycle:
    // 1st press -> auto-aim heading lock
    // 2nd press -> shoot (quick3 or ordered), then return to manual
    private enum AimState { MANUAL, AUTO_AIM_READY, SHOOTING }
    private AimState aimState = AimState.MANUAL;

    // -------------------- Timers --------------------
    private Timer teleOpTimer1;
    private double teleOpTime;

    // -------------------- Trigger Edge / Debounce --------------------
    private boolean rtHeld = false;      // debounced press
    private boolean rtPrevHeld = false;  // previous loop

    // Left-trigger: tap = stop feeders, hold = reverse feeders
    private boolean ltHeld = false;
    private double ltPressStartTime = 0.0;
    private static final double LT_PRESS_TH = 0.55;
    private static final double LT_RELEASE_TH = 0.45;
    private static final double LT_HOLD_SEC = 0.30;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        robotContext = new RobotContext();
        startingPose = robotContext.getStartingPose();

        controller = new PIDFController(new PIDFCoefficients(1.5, 0.0, 0.05, 0.033));
        controller.setCoefficients(new PIDFCoefficients(1.5, 0.0, 0.05, 0.033));

        follower.setStartingPose(startingPose);
        follower.update();

        intakeController = new Intake(hardwareMap);
        shotController = new Outtake(hardwareMap);
        ballColors = new colorDetector(hardwareMap);

        teleOpTimer1 = new Timer();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();

        aimState = AimState.MANUAL;
        manualDrive = true;

        intakeToggledOn = false;
        intakeController.spin(0);

        // Start clean: no lingering state machines or servo commands
        shotController.cancelAllShooting();
    }

    @Override
    public void loop() {
        follower.update();

        // Keep outtake updated each loop (shot accel detection + flywheel control)
        shotController.update(distanceFromGoal());

        // Update heading controller error for auto-aim mode
        setHeadingGoal();
        controller.updateError(getHeadingError());

        teleOpTime = teleOpTimer1.getElapsedTimeSeconds();

        // Driver 2: toggle shot mode QUICK <-> ORDERED (and hard-cancel if switching mid-action)
        updateShotModeToggle();

        // Driver 2: motif selection (21/22/23)
        updateMotifSelection();

        // Driver 1: intake controls (A toggle forward, B hold reverse)
        updateIntakeControls();

        // Driver 1: RT two-press cycle (aim then shoot)
        updateAimAndShootCycle();

        // Translation always manual; rotation depends on manualDrive
        if (!manualDrive) {
            // Auto heading lock to the goal while we translate manually
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, controller.run(), true);
        } else {
            // Full manual: right stick controls rotation
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }

        // Left-trigger behavior (tap -> stop feeders, hold -> reverse feeders)
        updateLeftTriggerShootBehavior();

        telemetry.addData("AimState", aimState);
        telemetry.addData("ShotMode", shotMode);
        telemetry.addData("manualDrive", manualDrive);
        telemetry.addData("intakeToggledOn", intakeToggledOn);
        telemetry.addData("motif", motif);
        telemetry.addData("positionGreen", positionGreen);
        telemetry.addData("headingGoal", headingGoal);
        telemetry.addData("headingError", headingError);
        telemetry.addData("distanceFromGoal", distanceFromGoal());
        telemetry.update();
    }

    // -------------------- Driver 1: Aim/Shoot Cycle --------------------
    private void updateAimAndShootCycle() {
        boolean rtPressed = rightTriggerPressedEvent();

        switch (aimState) {
            case MANUAL:
                // RT press #1: enter auto-aim (heading lock)
                if (rtPressed) {
                    manualDrive = false;
                    aimState = AimState.AUTO_AIM_READY;

                    // Safety: clear any leftover shooting before aiming
                    shotController.cancelAllShooting();
                }
                break;

            case AUTO_AIM_READY:
                // RT press #2: start the selected shooting mode
                if (rtPressed) {
                    aimState = AimState.SHOOTING;

                    if (shotMode == ShotMode.QUICK) {
                        // Quick 3-ball burst, unordered
                        shotController.startQuick3();
                    } else {
                        // Ordered: sample green position once at the start of the routine
                        ballColors.update();
                        positionGreen = ballColors.getGreenPosition();

                        shotController.setMotif(motif);
                        shotController.setPositionGreen(positionGreen);

                        // Ordered routine runs in SHOOTING until Outtake says we're done
                    }
                }
                break;

            case SHOOTING:
                // During SHOOTING the Outtake owns feeders + intake so we don't fight it
                if (shotMode == ShotMode.QUICK) {
                    shotController.runQuick3();
                    if (!shotController.isQuick3Active()) {
                        finishShootingReturnToManual();
                    }
                } else {
                    shotController.shootOrdered();
                    if (!shotController.isStillShooting()) {
                        finishShootingReturnToManual();
                    }
                }
                break;
        }
    }

    private void finishShootingReturnToManual() {
        // Ensure everything is off/clean for driver control
        shotController.cancelAllShooting();

        // Back to manual drive
        manualDrive = true;
        aimState = AimState.MANUAL;

        // Intake toggle state is preserved; intake output resumes next loop (unless B is held)
    }

    // -------------------- Driver 1: Intake Controls --------------------
    private void updateIntakeControls() {
        // Outtake controls intake while shooting; don't overwrite it here
        if (aimState == AimState.SHOOTING) return;

        // B is hold reverse intake (overrides toggle while held)
        if (gamepad1.b) {
            intakeController.spin(-1);
            return;
        }

        // A toggles intake forward ON/OFF on press
        boolean aNow = gamepad1.a;
        boolean aPressed = aNow && !aPrev;
        aPrev = aNow;

        if (aPressed) {
            intakeToggledOn = !intakeToggledOn;
        }

        intakeController.spin(intakeToggledOn ? 1 : 0);
    }

    // -------------------- Driver 2: Mode Toggle (QUICK <-> ORDERED) --------------------
    private void updateShotModeToggle() {
        // Mode toggle button: gamepad2.a
        boolean btnNow = gamepad2.a;
        boolean pressed = btnNow && !modeTogglePrev;
        modeTogglePrev = btnNow;

        if (!pressed) return;

        // Flip mode
        shotMode = (shotMode == ShotMode.QUICK) ? ShotMode.ORDERED : ShotMode.QUICK;

        // Hard reset anything mid-cycle so we never mix state machines
        shotController.cancelAllShooting();
        aimState = AimState.MANUAL;
        manualDrive = true;

        gamepad2.rumble(150);
    }

    // -------------------- Driver 2: Motif Selection --------------------
    private void updateMotifSelection() {
        // gp2.x -> 21 (GPP)
        // gp2.y -> 22 (PGP)
        // gp2.b -> 23 (PPG)
        if (gamepad2.xWasPressed()) {
            motif = 21;
            gamepad2.rumble(120);
        } else if (gamepad2.yWasPressed()) {
            motif = 22;
            gamepad2.rumble(120);
        } else if (gamepad2.bWasPressed()) {
            motif = 23;
            gamepad2.rumble(120);
        }
    }

    // -------------------- Left Trigger: tap stop, hold reverse --------------------
    private void updateLeftTriggerShootBehavior() {
        // Don’t interfere while Outtake is actively shooting
        if (aimState == AimState.SHOOTING) return;

        double lt = gamepad1.left_trigger;

        if (!ltHeld && lt > LT_PRESS_TH) {
            ltHeld = true;
            ltPressStartTime = teleOpTime;
        } else if (ltHeld && lt < LT_RELEASE_TH) {
            double heldSec = teleOpTime - ltPressStartTime;
            ltHeld = false;

            // Tap = stop feeders
            if (heldSec < LT_HOLD_SEC) {
                shotController.noShoot();
            }
        }

        // Hold = reverse feeders
        if (ltHeld && (teleOpTime - ltPressStartTime) >= LT_HOLD_SEC) {
            shotController.reverseShoot();
        }
    }

    // -------------------- RT Press Event (debounced) --------------------
    private boolean rightTriggerPressedEvent() {
        double rt = gamepad1.right_trigger;

        // Hysteresis to prevent flicker around threshold
        if (!rtHeld && rt > 0.55) rtHeld = true;
        else if (rtHeld && rt < 0.45) rtHeld = false;

        boolean pressedEvent = rtHeld && !rtPrevHeld;  // rising edge
        rtPrevHeld = rtHeld;

        return pressedEvent;
    }

    // -------------------- Geometry helpers --------------------
    private void setHeadingGoal() {
        headingGoal = Math.atan2(
                targetPose.getY() - follower.getPose().getY(),
                targetPose.getX() - follower.getPose().getX()
        );
    }

    private double distanceFromGoal() {
        return Math.sqrt(
                Math.pow(targetPose.getX() - follower.getPose().getX(), 2) +
                        Math.pow(targetPose.getY() - follower.getPose().getY(), 2)
        );
    }

    private double getHeadingError() {
        headingError =
                MathFunctions.getTurnDirection(follower.getPose().getHeading(), headingGoal) *
                        MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), headingGoal);
        return headingError;
    }
}