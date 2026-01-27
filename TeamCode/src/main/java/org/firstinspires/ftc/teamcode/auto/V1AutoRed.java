package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.TripleShot;

@Autonomous
public class V1AutoRed extends LinearOpMode {
    private DcMotor bl = null;
    private DcMotor fl = null;
    private DcMotor br = null;
    private DcMotor fr = null;
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private DcMotorSimple intake = null;

    private Outtake shotController;
    private drivetrain movementController;

    private boolean shotPressed = false;
    private double launcherVelocity = 1350;

    ElapsedTime autoTimer = new ElapsedTime();
    double autoTime = 0;

    private enum AutoState {
        SHOOT,
        MOVEBACK,
        MOVELEFT,
        IDLE;
    }

    AutoState autoState = AutoState.MOVEBACK;

    private Intake intakeController;

    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {



        bl = hardwareMap.get(DcMotor.class, "bl");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        intake = hardwareMap.get(DcMotorSimple.class, "intakeMotor");

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        shotController = new Outtake(hardwareMap);
        intakeController = new Intake(hardwareMap);

        waitForStart();
        autoTimer.reset();

        while(opModeIsActive()) {
            shotController.update(launcherVelocity);
            switch (autoState) {
                case MOVEBACK:
                    autoTime = autoTimer.seconds();
                    setAllPower(-.5);
                    if (autoTime > 2.4) {
                        autoTimer.reset();
                        autoState = AutoState.SHOOT;
                    }
                    break;
                case SHOOT:
                    autoTime = autoTimer.seconds();

                    setAllPower(0);

                    if (autoTime < 3) {
                        shotController.shootLeft();
                    } else if (autoTime < 9) {
                        shotController.shootRight();
                        intakeController.spin(1);
                    } else if (autoTime > 9) {
                        intakeController.spin(0);
                        autoState = AutoState.MOVELEFT;
                    }
                    break;
                case MOVELEFT:
                    autoTimer.reset();
                    autoTime = autoTimer.seconds();
                    bl.setPower(-.5);
                    br.setPower(.5);
                    fr.setPower(-.5);
                    fl.setPower(.5);
                    if (autoTime > 1.1) {
                        autoState = AutoState.IDLE;
                    }
                    break;
                case IDLE:
                    setAllPower(0);
                    break;
            }
            telemetry.addData("LaunchState", autoState);
            telemetry.addData("AutoTime", autoTime);
            telemetry.update();
        }
    }
    private void setAllPower(double pow) {
        bl.setPower(0.85 * pow);
        br.setPower(pow);
        fr.setPower(pow);
        fl.setPower(0.85 * pow);
    }
}
