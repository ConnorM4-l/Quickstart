package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.VelocitySolver;

@Configurable
public class Outtake {
    ElapsedTime launcherTimer = new ElapsedTime();

    private flywheel launcher;
    private feeders feed;
    private VelocitySolver velocitySolver;

    private double launcherTime = 0;

    private boolean stillShooting = true;

    private enum LaunchingState {
        SPIN,
        SHOOT1,
        BREAK1,
        SHOOT2,
        BREAK2,
        SHOOT3;
    }
    public static double timeShot = 5;
    public static double timeBetween;

    LaunchingState launchingState = LaunchingState.SPIN;

    public Outtake(HardwareMap hardwareMap) {
        launcher = new flywheel(hardwareMap);
        feed = new feeders(hardwareMap);

        velocitySolver = new VelocitySolver();

        launcherTimer.reset();
    }

    public void update(double distanceFromGoal) {
        double velocityRequested = velocitySolver.getVelocity(distanceFromGoal);

        launcher.update(distanceFromGoal);
        launcherTime = launcherTimer.seconds();
    }
    public void shootLeft() {
        feed.update(true, false);
    }
    public void shootRight() {
        feed.update(false, true);
    }
    public void shootBoth() { feed.update(true, true); }
    public void noShoot() { feed.update(false, false); }

    public boolean shootTwoThenOne() {
        switch (launchingState) {
            case SPIN:
                if (Math.abs(getErr()) < 100) {
                    launcherTimer.reset();
                    shootBoth();
                    launchingState = LaunchingState.SHOOT3;
                }
                break;
            case SHOOT3:
                launcherTime = launcherTimer.seconds();
                if (launcherTime > timeShot) {
                    launcherTimer.reset();
                    noShoot();
                    launchingState = LaunchingState.SPIN;
                    //this will make the loop stop running
                    return true;
                }
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

    public LaunchingState launchingState() {
        return launchingState;
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

    public double getRightAcceleration() {
        return launcher.getRightAcceleration();
    }
}
