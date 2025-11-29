package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {
    ElapsedTime launcherTimer = new ElapsedTime();

    private flywheel launcher;
    private feeders feed;

    private double launcherTime = 0;

    private double timeShot;
    private double timeBetween;

    private enum LaunchingState {
        SPIN,
        SHOOT1,
        BREAK1,
        SHOOT2,
        BREAK2,
        SHOOT3;
    }

    LaunchingState launchingState = LaunchingState.SPIN;

    public Outtake(HardwareMap hardwareMap, double tShot, double tBetween) {
        launcher = new flywheel(hardwareMap);
        feed = new feeders(hardwareMap);

        launcherTimer.reset();

        timeShot = tShot;
        timeBetween = tBetween;
    }

    public void update(double velocityRequested, boolean shotRequested, double timeShot, double timeBetween) {
        launcher.update(velocityRequested);
        launcherTime = launcherTimer.seconds();


    }
    public void LRRShoot(boolean aligned) {
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
                    launchingState = LaunchingState.SHOOT2;
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
