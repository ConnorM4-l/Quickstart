package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "SecondAutoRed")
public class SecondAutoRed extends LinearOpMode {
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
    private double launcherVelocity = 2650;

    ElapsedTime autoTimer = new ElapsedTime();
    double autoTime = 0;

    private enum AutoState {
        SHOOT,
        MOVEBACK,
        MOVERIGHT,
        IDLE;
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



        waitForStart();
        autoTimer.reset();

        while(opModeIsActive()) {
            switch (autoState) {
                case SHOOT:
                    shotController.update(true, false, launcherVelocity);
                    autoTime = autoTimer.seconds();

                    if (autoTime > 5) {
                        shotController.update(false, true, launcherVelocity);
                        autoTimer.reset();
                        autoState = AutoState.MOVEBACK;
                    }
                    break;
                case MOVEBACK:
                    autoTime = autoTimer.seconds();
                    setAllPower(-.5);
                    if (autoTime > 1) {
                        autoTimer.reset();
                        autoState = AutoState.MOVERIGHT;
                    }
                    break;
                case MOVERIGHT:
                    autoTime = autoTimer.seconds();
                    bl.setPower(-.5);
                    br.setPower(.5);
                    fr.setPower(-.5);
                    fl.setPower(.5);
                    if (autoTime > 1) {
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
        bl.setPower(pow);
        br.setPower(pow);
        fr.setPower(pow);
        fl.setPower(pow);
    }
}
