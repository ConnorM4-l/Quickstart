package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.localization.Encoder;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.VelocityPIDController;

@Configurable
public class TripleShot {
    private PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    TelemetryManager telemetryM = panelsTelemetry.getTelemetry();

    ElapsedTime launcherTimer = new ElapsedTime();
    ElapsedTime accTimer = new ElapsedTime();
    private double elapsedTime;

    private final VelocityPIDController launcherController = new VelocityPIDController();

    double LAUNCHER_TARGET_VELOCITY = 0;
    private double prevVelocity = 0;
    private double acceleration = 0;

    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    public static double FULL_SPEED = 1;

    double accTime = 0;

    int countShots = 0;

    boolean shotReady = false;

    private enum LaunchingState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    LaunchingState launchingState = LaunchingState.IDLE;

    private DcMotorEx leftLauncher;
    private DcMotorEx rightLauncher;
    private Encoder launcherEncoder;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    public TripleShot(HardwareMap hardwareMap) {
        leftLauncher = hardwareMap.get(DcMotorEx.class,"leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        leftFeeder = hardwareMap.get(CRServo.class,"leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class,"rightFeeder");

        launcherEncoder = new Encoder(leftLauncher);

        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherEncoder.setDirection(Encoder.REVERSE);

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        leftFeeder.setDirection(CRServo.Direction.FORWARD);
        rightFeeder.setDirection(CRServo.Direction.REVERSE);

        elapsedTime =Math.min(launcherTimer.seconds(),0.1);
        launcherTimer.reset();
        launcherEncoder.update();
    }



    public void update(boolean shotRequested, boolean shotEnd, double velocityRequested) {
        accTime = Math.min(accTimer.seconds(), 0.2);
        elapsedTime = Math.min(launcherTimer.seconds(), 0.1);

        launcherEncoder.update();


        switch (launchingState) {
            case IDLE:
                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);
                LAUNCHER_TARGET_VELOCITY = 0;
                setPower();
                if (shotRequested) {
                    launchingState = LaunchingState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                LAUNCHER_TARGET_VELOCITY = velocityRequested;
                setPower();
                if (Math.abs(launcherController.getErr()) < 100) {
                    launchingState = LaunchingState.LAUNCH;
                }
                break;
            case LAUNCH:
                setPower();
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                if (shotEnd) {
                    launchingState = LaunchingState.IDLE;
                }
                break;
            case LAUNCHING:
                setPower();
//                if (hasShot()) {
//                    if (countShots == 3) {
//                        countShots = 0;
//                        leftFeeder.setPower(STOP_SPEED);
//                        rightFeeder.setPower(STOP_SPEED);
//                        LAUNCHER_TARGET_VELOCITY = 0;
//                        launchingState = LaunchingState.IDLE;
//                    } else {
//                        countShots += 1;
//                        leftFeeder.setPower(STOP_SPEED);
//                        rightFeeder.setPower(STOP_SPEED);
//                        launchingState = LaunchingState.SPIN_UP;
//                    }
//                }
                break;
            }

        }


    private double launcherAcceleration() {
        return launcherController.getAcceleration();
    }

    private void setPower() {
        leftLauncher.setPower(launcherController.update(LAUNCHER_TARGET_VELOCITY, launcherEncoder.getDeltaPosition() * 60.0 / 28));
    }

    private boolean hasShot() {
        if (launcherAcceleration() > -1500) {
            accTimer.reset();
        }
        if (accTime > 0.1 && launcherAcceleration() <= -1500) {
            return true;
        } else {
            return false;
        }
    }
    public double getErr() {
        return launcherController.getErr();
    }
    public void CountShotsUp() {
        countShots += 1;
    }
    public void CountShotsDown() {
        countShots -= 1;
    }
    public int getCountShots() {
        return countShots;
    }

    public LaunchingState getLaunchingState() {
        return launchingState;
    }
    public double getVelocity() {
        return launcherController.getVelocity();
    }
    public double getAcceleration() {
        return launcherController.getAcceleration();
    }
    public boolean getHasShot() {
        return hasShot();
    }

}
