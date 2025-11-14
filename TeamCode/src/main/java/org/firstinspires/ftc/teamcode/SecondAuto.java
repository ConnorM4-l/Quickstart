package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class SecondAuto extends LinearOpMode {
    private DcMotor bl = null;
    private DcMotor fl = null;
    private DcMotor br = null;
    private DcMotor fr = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private TripleShot shotController;
    private Drivetrain movementController;

    private boolean shotPressed = false;
    private double launcherVelocity = 3500;

    ElapsedTime autoTimer = new ElapsedTime();
    double autoTime = 0;

    private enum AutoState {
        SHOOT,
        MOVEBACK,
        MOVELEFT;
    }

    AutoState autoState = AutoState.SHOOT;

    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        bl = hardwareMap.get(DcMotor.class, "bl");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shotController = new TripleShot(hardwareMap);

        autoTimer.reset();


        switch (autoState) {
            case SHOOT:
                shotController.update(true, false, launcherVelocity);

                autoTime = autoTimer.seconds();
                if (autoTime > 1) {
                    shotController.update(true, true, launcherVelocity);
                    autoTimer.reset();
                    autoState = AutoState.MOVEBACK;
                }
            case MOVEBACK:
                setAllPower(-1);
                if (autoTime > 1) {
                    autoState = AutoState.MOVELEFT;
                }
            case MOVELEFT:
                bl.setPower(1);
                br.setPower(-1);
                fr.setPower(1);
                fl.setPower(-1);
                if (autoTime > 1) {
                    setAllPower(0);
                }
        }
    }
    private void setAllPower(double pow) {
        bl.setPower(pow);
        br.setPower(pow);
        fr.setPower(pow);
        fl.setPower(pow);
    }
}
