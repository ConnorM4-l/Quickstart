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



    private enum LaunchingState {
        SPIN,
        SHOOT1,
        BREAK1,
        SHOOT2,
        BREAK2,
        SHOOT3;
    }
    public static double timeShot;
    public static double timeBetween;

    LaunchingState launchingState = LaunchingState.SPIN;

    public Outtake(HardwareMap hardwareMap, double tShot, double tBetween) {
        launcher = new flywheel(hardwareMap);
        feed = new feeders(hardwareMap);

        velocitySolver = new VelocitySolver();

        launcherTimer.reset();

        timeShot = tShot;
        timeBetween = tBetween;
    }

    public void update(double distanceFromGoal, double timeShot, double timeBetween) {
        double velocityRequested = velocitySolver.getVelocity(distanceFromGoal);

        launcher.update(velocityRequested);
        launcherTime = launcherTimer.seconds();
    }
    public void shootLeft() {
        feed.update(true, false);
    }
    public void shootRight() {
        feed.update(false, true);
    }
    public void shootBoth() {
        feed.update(true, true);
    }
    public boolean LRRShoot(boolean aligned) {
        switch (launchingState) {
            case SPIN:
                feed.update(false, false);
                if (aligned) {
                    launcherTimer.reset();
                    launchingState = LaunchingState.SHOOT1;
                }
                break;
            case SHOOT1:
                feed.update(true, false);
                if (launcherTime > timeShot) {
                    launcherTimer.reset();
                    launchingState = LaunchingState.BREAK1;
                }
                break;
            case BREAK1:
                feed.update(false, false);
                if (launcherTime > timeBetween && aligned) {
                    launcherTimer.reset();
                    launchingState = LaunchingState.SHOOT3;
                }
                break;
            case SHOOT2:
                feed.update(false, true);
                if (launcherTime > timeShot) {
                    launcherTimer.reset();
                    launchingState = LaunchingState.BREAK2;
                }
                break;
            case BREAK2:
                feed.update(false, false);
                if (launcherTime > timeBetween && aligned) {
                    launcherTimer.reset();
                    launchingState = LaunchingState.SHOOT2;
                }
                break;
            case SHOOT3:
                feed.update(false, true);
                if (launcherTime > timeShot) {
                    launcherTimer.reset();
                    launchingState = LaunchingState.SPIN;
                    return true;
                }
                break;
        }
    }

    public double getErr() {
        return launcher.getErr();
    }

    public double getVelocity() {
        return launcher.getVelocity();
    }

    public double getAcceleration() {
        return launcher.getAcceleration();
    }
}
