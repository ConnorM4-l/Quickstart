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

    private Pose targetPose = new Pose(12.844, 134.239, 0);

    private double headingError = 0;
    private double headingGoal = 0;

    // Manual vs heading-lock turning
    // true = right stick rotates, false = heading PID rotates
    private boolean manualDrive = true;

    // -------------------- Subsystems --------------------
    private Outtake shotController;
    private Intake intakeController;
    private RobotContext robotContext;
    private colorDetector ballColors;

    // -------------------- Modes --------------------
    // true = direct feeder control, false = auto shot (quick/ordered)
    private boolean manualShot = true;

    private enum ShotMode { QUICK, ORDERED }
    private ShotMode shotMode = ShotMode.QUICK;   // driver2 toggles this

    // -------------------- Auto Shot Flow --------------------
    // RT press #1 -> autoAim ON (manualDrive=false)
    // RT press #2 -> start shoot (quick or ordered), returns to manualDrive=true when done
    private enum AutoCycleState { IDLE, AIMING, SHOOTING }
    private AutoCycleState autoCycleState = AutoCycleState.IDLE;

    // Driver 2 selected order (Limelight motif): 21=GPP, 22=PGP, 23=PPG
    private int motif = 23;
    private int positionGreen = 0;   // 1 left, 2 right, 3 back, 0 unknown

    // -------------------- Intake Toggle --------------------
    private boolean intakeOn = false;
    private boolean aPrev = false;

    // -------------------- Timers --------------------
    private Timer teleOpTimer1;
    private double teleOpTime;

    // -------------------- RT edge detect (gp1) --------------------
    private boolean rtHeld = false;
    private boolean rtPrevHeld = false;

    // -------------------- Driver1 X/Y press edge detect (toggle manualDrive) --------------------
    private boolean xPrev = false;
    private boolean yPrev = false;


    // -------------------- LT tap/hold for feeder reverse --------------------
    private boolean ltHeld = false;
    private double ltPressStartTime = 0.0;
    private static final double LT_PRESS_TH = 0.55;
    private static final double LT_RELEASE_TH = 0.45;
    private static final double LT_HOLD_SEC = 0.30;

    // -------------------- START edge detect (manualShot toggle) --------------------
    private boolean startPrev = false;

    // -------------------- Driver2 shotMode toggle edge detect --------------------
    private boolean modeTogglePrev = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        robotContext = new RobotContext();
        startingPose = robotContext.getStartingPose();

        follower.setStartingPose(startingPose);
        follower.update();

        controller = new PIDFController(new PIDFCoefficients(1.5, 0.0, 0.05, 0.033));
        controller.setCoefficients(new PIDFCoefficients(1.5, 0.0, 0.05, 0.033));

        intakeController = new Intake(hardwareMap);
        shotController = new Outtake(hardwareMap);
        ballColors = new colorDetector(hardwareMap);

        teleOpTimer1 = new Timer();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();

        manualDrive = true;                 // default: manual turning
        manualShot = true;                  // default: manual feeder control
        autoCycleState = AutoCycleState.IDLE;

        intakeOn = false;
        intakeController.spin(0);

        shotController.cancelAllShooting();
    }

    @Override
    public void loop() {
        follower.update();
        shotController.update(distanceFromGoal());

        // Keep our heading goal updated every loop
        setHeadingGoal();
        controller.updateError(getHeadingError());

        teleOpTime = teleOpTimer1.getElapsedTimeSeconds();

        // ---- Driver2 toggles QUICK/ORDERED shot mode ----
        updateShotModeToggle();

        // ---- Driver2 selects motif (ordered shooting) ----
        updateMotifSelect();

        // ---- Emergency cancel (BACK) ----
        if (gamepad1.back) {
            cancelEverythingToManual();
        }

        // ---- Intake: A toggles forward ON/OFF, B holds reverse ----
        updateIntakeControls();

        // ---- ManualDrive is now forced by X/Y ----
        updateManualDriveXYToggle();

        // ---- ManualShot/AutoShot is now toggled by START ----
        updateManualShotToggleStart();

        // ---- Apply drive command ----
        applyDrive();

        // ---- Shooting control ----
        updateShooting();

        // ---- Feeder reverse LT tap/hold (still works in any mode) ----
        updateLeftTriggerFeedReverse();

        telemetry.addData("manualDrive", manualDrive);
        telemetry.addData("manualShot", manualShot);
        telemetry.addData("shotMode", shotMode);
        telemetry.addData("autoCycleState", autoCycleState);
        telemetry.addData("motif", motif);
        telemetry.addData("positionGreen", positionGreen);
        telemetry.addData("headingError", headingError);
        telemetry.addData("distanceFromGoal", distanceFromGoal());
        telemetry.update();
    }

    // ============================ DRIVE ============================

    private void applyDrive() {
        if (!manualDrive) {
            // Heading lock turning
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    controller.run(),
                    true
            );
        } else {
            // Manual turning
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );
        }
    }

    // X sets Manual Drive ON, Y sets Manual Drive OFF (tap-to-toggle behavior)
    private void updateManualDriveXYToggle() {
        // Don’t allow changes during the auto cycle (aiming/shooting)
        if (autoCycleState != AutoCycleState.IDLE) {
            xPrev = gamepad1.x;
            yPrev = gamepad1.y;
            return;
        }

        boolean xNow = gamepad1.x;
        boolean yNow = gamepad1.y;

        boolean xPressed = xNow && !xPrev;
        boolean yPressed = yNow && !yPrev;

        xPrev = xNow;
        yPrev = yNow;

        if (xPressed) manualDrive = true;   // manual turn (right stick)
        if (yPressed) manualDrive = false;  // heading lock (PID)
    }


    private void updateManualShotToggleStart() {
        boolean startNow = gamepad1.start;
        boolean startPressed = startNow && !startPrev;
        startPrev = startNow;

        if (!startPressed) return;

        // Flip between manual feeder control and auto shot routines
        manualShot = !manualShot;

        // If we just switched INTO manualShot, kill any auto routine immediately
        if (manualShot) {
            autoCycleState = AutoCycleState.IDLE;
            manualDrive = true;
            shotController.cancelAllShooting();
        } else {
            // Switched INTO auto shot mode: also hard-cancel any manual feeder hold
            shotController.noShoot();
            // Keep manualDrive as-is; driver will use RT press #1 to enter aiming (manualDrive=false)
        }
    }

    // ============================ INTAKE ============================

    private void updateIntakeControls() {
        // B is hold-to-reverse (unjam/spit)
        if (gamepad1.b) {
            intakeController.spin(-1);
            return;
        }

        // A toggles forward intake ON/OFF
        boolean aNow = gamepad1.a;
        boolean aPressed = aNow && !aPrev;
        aPrev = aNow;

        if (aPressed) {
            intakeOn = !intakeOn;
        }

        intakeController.spin(intakeOn ? 1 : 0);
    }

    // ============================ SHOOTING ============================

    private void updateShooting() {
        // Manual feeder mode: RT=both, LB=left, RB=right (all while held).
        // Alignment gate + D-pad left override remain.
        if (manualShot) {
            // If we’re mid auto-cycle, manualShot should not run the feeders
            if (autoCycleState != AutoCycleState.IDLE) {
                shotController.noShoot();
                return;
            }

            boolean okToShoot = (Math.abs(getHeadingError()) < 0.1) || gamepad1.dpad_left;

            if (okToShoot) {
                if (gamepad1.right_trigger > 0.5) {
                    shotController.shootBoth();
                } else if (gamepad1.left_bumper) {
                    shotController.shootLeft();
                } else if (gamepad1.right_bumper) {
                    shotController.shootRight();
                } else {
                    shotController.noShoot();
                }
            } else {
                shotController.noShoot();
            }

            return;
        }

        // Auto-shot branch (manualShot == false):
        // RT press 1: enter aiming (manualDrive=false)
        // RT press 2: start shooting (quick or ordered)
        // After shooting finishes: return to manualDrive=true and IDLE
        boolean rtPressed = rightTriggerPressedEvent();

        switch (autoCycleState) {
            case IDLE:
                if (rtPressed) {
                    // enter autoAim
                    manualDrive = false;
                    autoCycleState = AutoCycleState.AIMING;
                }
                break;

            case AIMING:
                if (rtPressed) {
                    // start shooting
                    beginAutoShooting();
                    autoCycleState = AutoCycleState.SHOOTING;
                }
                break;

            case SHOOTING:
                // run whichever shot routine is selected
                if (shotMode == ShotMode.QUICK) {
                    shotController.runQuick3();
                    if (!shotController.isQuick3Active()) {
                        finishAutoCycleToManual();
                    }
                } else {
                    shotController.shootOrdered();
                    if (!shotController.isStillShooting()) {
                        shotController.noShoot();
                        finishAutoCycleToManual();
                    }
                }
                break;
        }
    }

    private void beginAutoShooting() {
        // Stop direct feeder holds before the state machine takes over
        shotController.noShoot();

        if (shotMode == ShotMode.QUICK) {
            // quick burst (unordered)
            shotController.startQuick3();
        } else {
            // ordered: read color, set motif + position
            ballColors.update();
            positionGreen = ballColors.getGreenPosition();

            shotController.setMotif(motif);
            shotController.setPositionGreen(positionGreen);
            // shootOrdered() advances starting next loop
        }
    }

    private void finishAutoCycleToManual() {
        manualDrive = true;
        autoCycleState = AutoCycleState.IDLE;
    }

    private void cancelEverythingToManual() {
        // Hard stop everything that could keep moving hardware
        shotController.cancelAllShooting();
        intakeOn = false;
        intakeController.spin(0);

        // Return to manual
        manualDrive = true;
        autoCycleState = AutoCycleState.IDLE;
        manualShot = true; // safest default after a hard cancel
    }

    // ============================ FEEDER REVERSE (LT tap/hold) ============================

    private void updateLeftTriggerFeedReverse() {
        double lt = gamepad1.left_trigger;

        if (!ltHeld && lt > LT_PRESS_TH) {
            ltHeld = true;
            ltPressStartTime = teleOpTime;
        } else if (ltHeld && lt < LT_RELEASE_TH) {
            double heldSec = teleOpTime - ltPressStartTime;
            ltHeld = false;

            if (heldSec < LT_HOLD_SEC) {
                shotController.noShoot(); // tap = stop feeding
            }
        }

        if (ltHeld && (teleOpTime - ltPressStartTime) >= LT_HOLD_SEC) {
            shotController.reverseShoot(); // hold = reverse feeders
        }
    }

    // ============================ DRIVER 2: MODE + MOTIF ============================

    private void updateShotModeToggle() {
        // Driver2 A toggles QUICK <-> ORDERED (only when gp2 RT is not held)
        if (gamepad2.right_trigger > 0.5) return;

        boolean btnNow = gamepad2.a;
        boolean pressed = btnNow && !modeTogglePrev;
        modeTogglePrev = btnNow;

        if (pressed) {
            shotMode = (shotMode == ShotMode.QUICK) ? ShotMode.ORDERED : ShotMode.QUICK;

            // Cancel any active cycle so we don’t mix state machines mid-flight
            autoCycleState = AutoCycleState.IDLE;
            manualDrive = true;
            shotController.cancelAllShooting();
        }
    }

    private void updateMotifSelect() {
        // gp2.x -> 21 (GPP), gp2.y -> 22 (PGP), gp2.b -> 23 (PPG)
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

        // Driver2 velocity failsafe block:
        // (Kept as comment placeholder—paste your exact block here if you want it active in this file)
        // if (gamepad2.right_trigger > 0.5) {
        //     if (gamepad2.dpad_up) { targetVelocity = 1400; }
        //     else if (gamepad2.dpad_down) { targetVelocity = 1600; }
        //     else if (gamepad2.dpad_left) { targetVelocity = 1240; }
        //     if (gamepad2.leftBumperWasPressed()) targetVelocity -= 10;
        //     else if (gamepad2.rightBumperWasPressed()) targetVelocity += 10;
        // }
    }

    // ============================ RT PRESS EVENT ============================

    // Debounced RT "press event" for driver1
    public boolean rightTriggerPressedEvent() {
        double rt = gamepad1.right_trigger;

        if (!rtHeld && rt > 0.55) rtHeld = true;
        else if (rtHeld && rt < 0.45) rtHeld = false;

        boolean pressedEvent = rtHeld && !rtPrevHeld;
        rtPrevHeld = rtHeld;

        return pressedEvent;
    }

    // ============================ HEADING HELPERS ============================

    public void setHeadingGoal() {
        headingGoal = Math.atan2(
                targetPose.getY() - follower.getPose().getY(),
                targetPose.getX() - follower.getPose().getX()
        );
    }

    public double getHeadingError() {
        headingError =
                MathFunctions.getTurnDirection(follower.getPose().getHeading(), headingGoal) *
                        MathFunctions.getSmallestAngleDifference(follower.getPose().getHeading(), headingGoal);
        return headingError;
    }

    public double distanceFromGoal() {
        return Math.sqrt(
                Math.pow(targetPose.getX() - follower.getPose().getX(), 2) +
                        Math.pow(targetPose.getY() - follower.getPose().getY(), 2)
        );
    }

}
