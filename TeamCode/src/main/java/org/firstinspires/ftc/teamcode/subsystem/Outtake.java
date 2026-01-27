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

    private boolean stillShooting = true;

    private boolean prevLeft = false;
    private boolean left = false;

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
        SHOOT_BOTH;
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

        motif = 21;
        positionGreen = 1;
    }

    public void update(double distanceFromGoal) {
        //double velocityRequested = velocitySolver.getVelocity(distanceFromGoal);


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
        int order = new DecideOrder().order(motif, positionGreen);
        switch (launchingState) {

            case SPIN:
                stillShooting = true;

                if (Math.abs(getErr()) < 100) {
                    launcherTimer.reset();
                    shotsLeft = 0;
                    shotsRight = 0;

                    // order comes from DecideOrder.order(motif, positionGreen)
                    if (order == -1) {
                        // SHOOT_BOTH = run both CR servos continuously; stop after 3 shot events
                        totalShots = 0;          // IMPORTANT reset
                        shootBoth();
                        launcherTimer.reset();
                        launchingState = LaunchingState.SHOOT_BOTH;

                    } else if (order == 1) { // LL
                        shootLeft();
                        launcherTimer.reset();
                        launchingState = LaunchingState.SHOOTLL;

                    } else if (order == 2) { // RR
                        shootRight();
                        launcherTimer.reset();
                        launchingState = LaunchingState.SHOOTRR;

                    } else if (order == 3) { // LR
                        shootLeft();
                        launcherTimer.reset();
                        launchingState = LaunchingState.SHOOTLR;

                    } else if (order == 4) { // RL
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
                }

                if (hasShotLeft() || launcherTime > timeShot) {
                    shotsLeft++;
                    launcherTimer.reset();
                    if (shotsLeft < 2) shootLeft();
                }

                if (shotsLeft >= 2) {
                    shootRight();
                    launcherTimer.reset();
                    launchingState = LaunchingState.SHOOTR;
                }
                break;

            case SHOOTRR:
                if (enteredState()) {
                    launcherTimer.reset();
                    shotsRight = 0;
                }

                if (hasShotRight() || launcherTime > timeShot) {
                    shotsRight++;
                    launcherTimer.reset();
                    if (shotsRight < 2) shootRight();
                }

                if (shotsRight >= 2) {
                    shootLeft();
                    launcherTimer.reset();
                    launchingState = LaunchingState.SHOOTL;
                }
                break;

            case SHOOTLR:
                if (enteredState()) {
                    launcherTimer.reset();
                    shotsLeft = 0;
                    shotsRight = 0;
                }

                // First: wait for left shot to complete, then command right
                if (shotsLeft == 0 && (hasShotLeft() || launcherTime > timeShot)) {
                    shotsLeft = 1;
                    launcherTimer.reset();
                    shootRight();
                }
                // Then: wait for right shot to complete, then go to SHOOT_BOTH (3rd ball, order doesn't matter)
                else if (shotsLeft == 1 && shotsRight == 0 && (hasShotRight() || launcherTime > timeShot)) {
                    shotsRight = 1;
                    launcherTimer.reset();

                    totalShots = 0;      // we're going to shoot 3 balls in SHOOT_BOTH mode
                    shootBoth();         // spins both CR servos continuously
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

                // First: wait for right shot to complete, then command left
                if (shotsRight == 0 && (hasShotRight() || launcherTime > timeShot)) {
                    shotsRight = 1;
                    launcherTimer.reset();
                    shootLeft();
                }
                // Then: wait for left shot to complete, then go to SHOOT_BOTH (3rd ball, order doesn't matter)
                else if (shotsRight == 1 && shotsLeft == 0 && (hasShotLeft() || launcherTime > timeShot)) {
                    shotsLeft = 1;
                    launcherTimer.reset();

                    totalShots = 0;
                    shootBoth();
                    launcherTimer.reset();
                    launchingState = LaunchingState.SHOOT_BOTH;
                }
                break;

            case SHOOT_BOTH:
                if (enteredState()) {
                    launcherTimer.reset();
                    totalShots = 0;     // IMPORTANT reset
                }

                boolean lEvt = shotLeftEvent();   // edge-detect
                boolean rEvt = shotRightEvent();  // edge-detect

                if (lEvt) totalShots++;
                if (rEvt) totalShots++;

                // Stop after 3 total balls have been shot
                if (totalShots >= 3) {
                    noShoot();
                    launchingState = LaunchingState.SPIN;
                    stillShooting = false;
                    break;
                }

                // Safety timeout (jam / missed sensor events)
                if (launcherTime > (timeShot * 3.0)) {
                    noShoot();
                    launchingState = LaunchingState.SPIN;
                    stillShooting = false;
                }
                break;

            case SHOOTL:
                if (enteredState()) launcherTimer.reset();

                if (hasShotLeft() || launcherTime > timeShot) {
                    launchingState = LaunchingState.SPIN;
                    stillShooting = false;
                    noShoot();
                }
                break;

            case SHOOTR:
                if (enteredState()) launcherTimer.reset();

                if (hasShotRight() || launcherTime > timeShot) {
                    launchingState = LaunchingState.SPIN;
                    stillShooting = false;
                    noShoot();
                }
                break;
        }

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
        if (getLeftAcceleration() > -1000) {
            leftLauncherTimer.reset();
        } else if (leftLauncherTime > 0.1 && getLeftAcceleration() < -1000) {
            return true;
        }
        return false;
    }

    private boolean hasShotRight() {
        if (getRightAcceleration() > -1000) {
            rightLauncherTimer.reset();
        } else if (rightLauncherTime > 0.1 && getRightAcceleration() < -1000) {
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
