package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DecideOrder;
import org.firstinspires.ftc.teamcode.util.VelocitySolver;

@Configurable
public class Outtake {
    ElapsedTime launcherTimer = new ElapsedTime();
    ElapsedTime leftLauncherTimer = new ElapsedTime();
    ElapsedTime rightLauncherTimer = new ElapsedTime();


    private flywheel launcher;
    private feeders feed;
    private VelocitySolver velocitySolver;
    private Intake intake;


    private double launcherTime = 0;
    private double leftLauncherTime = 0;
    private double rightLauncherTime = 0;
    private double ballsInRobot = 3;

    private int shotsLeft;
    private int shotsRight;

    private int totalShots = 0;

    // edge detect so we don't double-count
    private boolean prevShotL = false;
    private boolean prevShotR = false;

    private boolean stillShooting = false;

    private boolean prevLeft = false;
    private boolean left = false;

    private double velocityRequested = 0;

    // ---------------- QUICK 3-SHOT MODE ----------------
    private boolean quick3Active = false;        // state machine running
    private boolean quick3IntakeStarted = false; // ensure intake start happens once
    private int quick3Shots = 0;                // how many balls detected



    private boolean bothLeftDone = false;
    private boolean bothRightDone = false;

    private double bothStartTime = 0;

    private int motif;

    private int positionGreen;

    private LaunchingState prevLaunchingState = null;

    private enum LaunchingState {
        SPIN,
        SHOOT1,
        BREAK1,
        SHOOT2,
        BREAK2,
        SHOOT3,
        SHOOTLL,
        SHOOTRR,
        SHOOTLR,
        SHOOTRL,
        SHOOTL,
        SHOOTR,
        SHOOT_BOTH,
        SHOOT_THREE_QUICK;
    }
    //get this as low as possible, grip tape?
    public static double timeShot = 3;
    public static double timeBetween;

    LaunchingState launchingState = LaunchingState.SPIN;

    public Outtake(HardwareMap hardwareMap) {
        launcher = new flywheel(hardwareMap);
        feed = new feeders(hardwareMap);
        intake = new Intake(hardwareMap);

        velocitySolver = new VelocitySolver();

        launcherTimer.reset();
        leftLauncherTimer.reset();
        rightLauncherTimer.reset();

        motif = 0;
        positionGreen = 0;
    }

    public void update(double distanceFromGoal) {
        velocityRequested = velocitySolver.getVelocity(distanceFromGoal);


        launcher.update(distanceFromGoal);
        launcherTime = launcherTimer.seconds();
        rightLauncherTime = rightLauncherTimer.seconds();
        leftLauncherTime = leftLauncherTimer.seconds();
    }
    public void shootLeft() {
        feed.update(true, false);
    }
    public void shootRight() {
        feed.update(false, true);
    }
    public void shootBoth() { feed.update(true, true); }
    public void noShoot() { feed.update(false, false); }
    public void reverseShoot() { feed.reverse(); }

    /*
    motif:
    Blue Goal: 20
    Motif GPP: 21
    Motif PGP: 22
    Motif PPG: 23
    Red Goal: 24

    positionGreen:
    1 = left
    2 = right
    3 = back

    return:
    1: LL
    2: RR
    3: LR
    4: RL
     */
    public void setPositionGreen(int pos) {
        positionGreen = pos;
    }


    public void shootOrdered() {

        switch (launchingState) {

            case SPIN:
                stillShooting = true;

                if (Math.abs(getErr()) < 100) {
                    launcherTimer.reset();
                    shotsLeft = 0;
                    shotsRight = 0;

                    prevShotL = false;
                    prevShotR = false;

                    int order = new DecideOrder().order(motif, positionGreen);

                    if (order == -1) {
                        // shoot all 3, order doesn't matter
                        totalShots = 0;
                        shootBoth();
                        intake.spin(1);
                        launcherTimer.reset();
                        launchingState = LaunchingState.SHOOT_BOTH;

                    } else if (order == 1) { // LL
                        // first left shot: do NOT need intake yet
                        intake.spin(0);
                        shootLeft();
                        launcherTimer.reset();
                        launchingState = LaunchingState.SHOOTLL;

                    } else if (order == 2) { // RR
                        // first right shot: do NOT need intake yet
                        intake.spin(0);
                        shootRight();
                        launcherTimer.reset();
                        launchingState = LaunchingState.SHOOTRR;

                    } else if (order == 3) { // LR
                        intake.spin(0);
                        shootLeft();
                        launcherTimer.reset();
                        launchingState = LaunchingState.SHOOTLR;

                    } else if (order == 4) { // RL
                        intake.spin(0);
                        shootRight();
                        launcherTimer.reset();
                        launchingState = LaunchingState.SHOOTRL;
                    }
                }
                break;

            case SHOOTLL:
                if (enteredState()) {
                    launcherTimer.reset();
                    shotsLeft = 0;
                    // second left ball comes from back -> intake must run after first shot
                }

                // FIRST left shot finishes -> start intake + command SECOND left shot
                if (shotsLeft == 0 && (shotLeftEvent() || launcherTime > timeShot)) {
                    shotsLeft = 1;
                    launcherTimer.reset();
                    intake.spin(1);      // NEW: pull back ball to left
                    // command second left shot feed should still be spinning
                }
                // SECOND left shot finishes -> go to SHOOT_BOTH for final ball
                else if (shotsLeft == 1 && (shotLeftEvent() || launcherTime > timeShot)) {
                    shotsLeft = 2;
                    launcherTimer.reset();

                    totalShots = 2;      // already fired 2 balls
                    shootBoth();         // run both feeders for last ball
                    intake.spin(1);      // keep pushing back ball forward
                    launcherTimer.reset();
                    launchingState = LaunchingState.SHOOT_BOTH;
                }
                break;

            case SHOOTRR:
                if (enteredState()) {
                    launcherTimer.reset();
                    shotsRight = 0;
                    // second right ball comes from back -> intake must run after first shot
                }

                // FIRST right shot finishes -> start intake + command SECOND right shot
                if (shotsRight == 0 && (shotRightEvent() || launcherTime > timeShot)) {
                    shotsRight = 1;
                    launcherTimer.reset();
                    intake.spin(1);      // NEW: pull back ball to right
                    shootRight();        // command second right shot
                }
                // SECOND right shot finishes -> go to SHOOT_BOTH for final ball
                else if (shotsRight == 1 && (shotRightEvent() || launcherTime > timeShot)) {
                    shotsRight = 2;
                    launcherTimer.reset();

                    totalShots = 2;      // already fired 2 balls
                    shootBoth();
                    intake.spin(1);
                    launcherTimer.reset();
                    launchingState = LaunchingState.SHOOT_BOTH;
                }
                break;

            case SHOOTLR:
                if (enteredState()) {
                    launcherTimer.reset();
                    shotsLeft = 0;
                    shotsRight = 0;
                }

                // wait for left shot to complete, then command right
                if (shotsLeft == 0 && (shotLeftEvent() || launcherTime > timeShot)) {
                    shotsLeft = 1;
                    launcherTimer.reset();
                    shootRight();
                }
                // wait for right shot to complete, then go to SHOOT_BOTH for 3rd
                else if (shotsLeft == 1 && shotsRight == 0 && (shotRightEvent() || launcherTime > timeShot)) {
                    shotsRight = 1;
                    launcherTimer.reset();

                    totalShots = 2;
                    shootBoth();
                    intake.spin(1);          // push the back ball for the last shot
                    launcherTimer.reset();
                    launchingState = LaunchingState.SHOOT_BOTH;
                }
                break;

            case SHOOTRL:
                if (enteredState()) {
                    launcherTimer.reset();
                    shotsLeft = 0;
                    shotsRight = 0;
                }

                // wait for right shot to complete, then command left
                if (shotsRight == 0 && (shotRightEvent() || launcherTime > timeShot)) {
                    shotsRight = 1;
                    launcherTimer.reset();
                    shootLeft();
                }
                // wait for left shot to complete, then go to SHOOT_BOTH for 3rd
                else if (shotsRight == 1 && shotsLeft == 0 && (shotLeftEvent() || launcherTime > timeShot)) {
                    shotsLeft = 1;
                    launcherTimer.reset();

                    totalShots = 2;
                    shootBoth();
                    intake.spin(1);
                    launcherTimer.reset();
                    launchingState = LaunchingState.SHOOT_BOTH;
                }
                break;

            case SHOOT_BOTH:
                if (enteredState()) {
                    launcherTimer.reset();
                    // totalShots is NOT reset here (LL/RR/LR/RL enter with totalShots=2)
                    // intake should already be on, but keep it on
                    intake.spin(1);
                }

                boolean lEvt = shotLeftEvent();
                boolean rEvt = shotRightEvent();

                if (lEvt) totalShots++;
                if (rEvt) totalShots++;

                if (totalShots >= 3) {
                    noShoot();
                    intake.spin(0);
                    launchingState = LaunchingState.SPIN;
                    stillShooting = false;
                    break;
                }

                if (launcherTime > (timeShot * 3.0)) {
                    noShoot();
                    intake.spin(0);
                    launchingState = LaunchingState.SPIN;
                    stillShooting = false;
                }
                break;

            case SHOOTL:
                if (enteredState()) launcherTimer.reset();

                if (shotLeftEvent() || launcherTime > timeShot) {
                    launchingState = LaunchingState.SPIN;
                    stillShooting = false;
                    noShoot();
                    intake.spin(0);
                }
                break;

            case SHOOTR:
                if (enteredState()) launcherTimer.reset();

                if (shotRightEvent() || launcherTime > timeShot) {
                    launchingState = LaunchingState.SPIN;
                    stillShooting = false;
                    noShoot();
                    intake.spin(0);
                }
                break;
        }
    }

    public void startQuick3() {
        if (quick3Active) return;      // start only once

        // reset counters + edge detect so we don't double-count old events
        quick3Active = true;
        quick3IntakeStarted = false;
        quick3Shots = 0;

        prevShotL = false;
        prevShotR = false;

        // start feeders NOW, intake stays off until first shot is detected
        shootBoth();
        intake.spin(0);
        launcherTimer.reset();
    }

    public void runQuick3() {
        if (!quick3Active) return;

        // count shot events
        boolean lEvt = shotLeftEvent();
        boolean rEvt = shotRightEvent();

        if (lEvt) quick3Shots++;
        if (rEvt) quick3Shots++;

        // after FIRST recorded shot, start intake ONCE
        if (!quick3IntakeStarted && quick3Shots >= 1) {
            intake.spin(1);
            quick3IntakeStarted = true;
        }

        // finish after 3 shots
        if (quick3Shots >= 3) {
            stopQuick3();
            return;
        }

        // safety timeout (optional): prevent infinite spin if sensors miss
        double t = launcherTimer.seconds();
        if (t > (timeShot * 3.0)) {
            stopQuick3();
        }
    }

    public void stopQuick3() {
        if (!quick3Active) return;  // stop only once

        noShoot();
        intake.spin(0);

        quick3Active = false;
    }

    public void cancelQuick3() {
        // same as stop, but safe to call anytime
        stopQuick3();
    }

    public boolean isQuick3Active() {
        return quick3Active;
    }


    public void cancelAllShooting() {
        // Stop any feeder/intake motion immediately
        noShoot();
        intake.spin(0);

        // Cancel Quick3 mode
        quick3Active = false;
        quick3IntakeStarted = false;
        quick3Shots = 0;

        // Cancel Ordered mode / any other shooting state machine
        stillShooting = false;
        launchingState = LaunchingState.SPIN;
        prevLaunchingState = null;

        // Clear shot edge-detect so old sensor states don't count as new shots
        prevShotL = false;
        prevShotR = false;

        // Clear counters
        shotsLeft = 0;
        shotsRight = 0;
        totalShots = 0;

        // Reset timers so you don't instantly timeout / instantly trigger hasShot logic
        launcherTimer.reset();
        leftLauncherTimer.reset();
        rightLauncherTimer.reset();
    }





    public LaunchingState launchingState() {
        return launchingState;
    }

    public void setMotif(int m) {
        motif = m;
    }

    public double getLeftErr() {
        return launcher.getLeftErr();
    }

    public double getRightErr() {
        return launcher.getRightErr();
    }

    public boolean isStillShooting() {
        return stillShooting;
    }

//    public void setLRR

    public double getLeftVelocity() {
        return launcher.getLeftVelocity();
    }

    public double getRightVelocity() {
        return launcher.getRightVelocity();
    }

    private double getErr() {
        return Math.max(Math.abs(launcher.getLeftErr()), Math.abs(launcher.getRightErr()));
    }


    public double getLeftAcceleration() {
        return launcher.getLeftAcceleration();
    }

    public double getBallsInRobot() {
        return ballsInRobot;
    }

    public double getVelocityRequested() { return velocityRequested; }


    public int getShotsLeft() {
        return shotsLeft;
    }

    public int getShotsRight() {
        return shotsRight;
    }

    private void reset() {
        launcherTimer.reset();
        bothLeftDone = false;
        bothRightDone = false;
        totalShots = 0;
    }


    private boolean shotLeftEvent() {
        boolean cur = hasShotLeft();
        boolean evt = cur && !prevShotL;
        prevShotL = cur;
        return evt;
    }

    private boolean shotRightEvent() {
        boolean cur = hasShotRight();
        boolean evt = cur && !prevShotR;
        prevShotR = cur;
        return evt;
    }

    private boolean hasShotLeft() {
        if (getLeftAcceleration() < -2000) {
            return true;
        }
        return false;
    }

    private boolean hasShotRight() {
        if (getRightAcceleration() < -2000) {
            return true;
        }
        return false;
    }

    public boolean hasShotThree() {
        if (getLeftAcceleration() > -1000) {
            leftLauncherTimer.reset();
        } else if (leftLauncherTime > 0.1 && getLeftAcceleration() < -1000) {
            ballsInRobot -= 1;
        }
        if (getRightAcceleration() < -1000) {
            rightLauncherTimer.reset();
        } else if (rightLauncherTime > 0.1) {
            ballsInRobot -= 1;
        }
        if (ballsInRobot == 0) {
            return true;
        }
        return false;
    }

    public double getRightAcceleration() {
        return launcher.getRightAcceleration();
    }

    private boolean enteredState() {
        boolean entered = (launchingState != prevLaunchingState);
        prevLaunchingState = launchingState;
        return entered;
    }


    public boolean shootTwoThenOne() {
        switch (launchingState) {
            case SPIN:
                //high error tolerance because of high time to shoot
                if (Math.abs(getErr()) < 100) {
                    launcherTimer.reset();
                    shootBoth();
                    intake.spin(1);
                    launchingState = LaunchingState.SHOOT3;
                }
                break;
            case SHOOT3:
                launcherTime = launcherTimer.seconds();
                if (launcherTime > timeShot) {
                    launchingState = LaunchingState.SPIN;
                    noShoot();
                    return true;
                }
                break;
            case SHOOT2:
                launcherTime = launcherTimer.seconds();
                if (launcherTime > timeShot ) {
                    launcherTimer.reset();
                    intake.spin(1);
                    launchingState = LaunchingState.SHOOT1;
                }
                break;
            case SHOOT1:
                launcherTime = launcherTimer.seconds();
                if (launcherTime > timeShot) {
                    launchingState = LaunchingState.SPIN;
                    noShoot();
                    //this will make the loop stop running
                    return true;
                }
                break;
        }
        return false;
    }

    public void LRRShoot(boolean aligned) {
        switch (launchingState) {
            case SPIN:
                noShoot();
                if (aligned) {
                    launcherTimer.reset();
                    launchingState = LaunchingState.SHOOT1;
                }
                stillShooting = true;
                break;
            case SHOOT1:
                shootLeft();
                launcherTime = launcherTimer.seconds();
                if (launcherTime > timeShot) {
                    launcherTimer.reset();
                    launchingState = LaunchingState.BREAK1;
                }
                break;
            case BREAK1:
                noShoot();
                launcherTime = launcherTimer.seconds();
                if (launcherTime > timeBetween && aligned) {
                    launcherTimer.reset();
                    launchingState = LaunchingState.SHOOT3;
                }
                break;
            case SHOOT2:
                shootRight();
                launcherTime = launcherTimer.seconds();
                if (launcherTime > timeShot) {
                    launcherTimer.reset();
                    launchingState = LaunchingState.BREAK2;
                }
                break;
            case BREAK2:
                noShoot();
                launcherTime = launcherTimer.seconds();
                if (launcherTime > timeBetween && aligned) {
                    launcherTimer.reset();
                    launchingState = LaunchingState.SHOOT2;
                }
                break;
            case SHOOT3:
                shootRight();
                launcherTime = launcherTimer.seconds();
                if (launcherTime > timeShot) {
                    launcherTimer.reset();
                    launchingState = LaunchingState.SPIN;
                    stillShooting = false;
                }
                break;
        }
    }

    public void shootOne() {
        switch (launchingState) {
            case SPIN:
                noShoot();
                launcherTimer.reset();
                launchingState = LaunchingState.SHOOT1;
                break;
            case SHOOT1:
                shootLeft();
                if (hasShotLeft()) {
                    launchingState = LaunchingState.SPIN;
                    stillShooting = false;
                }
                break;
        }
    }

    public void countShots() {
        switch (launchingState) {
            case SPIN:
                noShoot();
                if (Math.abs(getErr()) < 100) {
                    shootBoth();
                    launchingState = LaunchingState.SHOOT1;
                }
                break;
            case SHOOT1:
                if (hasShotLeft()) {
                    shotsLeft += 1;
                } else if (hasShotRight()) {
                    shotsRight += 1;
                }
                break;
        }

    }
}
