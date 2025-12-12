package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.limelight;

@Configurable
@TeleOp(name = "AimTesting")
public class AimTesting extends OpMode {

    private TelemetryManager telemetryM;

    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;

    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private boolean automatedDrive = false;

    private Pose targetPose = new Pose(-67, -67, 0);

    private double targetHeading = 0;

    public static double launcherVelocity = 1000;
    private Outtake shotController;
    private Intake intakeController;
    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(67, 67, Math.toRadians(45))); //needs to be in radians

        shotController = new Outtake(hardwareMap);

        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        shotController.update(launcherVelocity);

        targetHeading = desiredHeading();

        if (automatedDrive) {
            follower.turnToDegrees(targetHeading);
        }
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric (false for field centric)
        );
        if (gamepad1.right_bumper) {
            automatedDrive = true;
        } else if (gamepad1.left_bumper) {
            automatedDrive = false;
        }

        if (gamepad1.xWasPressed()) {
            shotController.shootLeft();
        }

        if (gamepad1.yWasPressed()) {
            shotController.shootRight();
        }

        if (gamepad1.aWasPressed()) {
            intakeController.spin(1);
        }
        if (gamepad1.bWasPressed()) {
            intakeController.spin(0);
        }



        telemetryM.addData("position", follower.getPose());
        telemetryM.addData("velocity", follower.getVelocity());
        telemetryM.addData("automatedDrive", automatedDrive);
        telemetryM.addData("is aligned", isAligned());
        telemetryM.addData("distance from goal", distanceFromGoal());
        telemetryM.addData("target heading", targetHeading);
        telemetryM.addData("current heading", follower.getHeading());
        telemetryM.update();
    }
    public double desiredHeading() {
        return Math.toDegrees(Math.atan2(targetPose.getY() - follower.getPose().getY(), targetPose.getX()) - follower.getPose().getX());
    }
    public double distanceFromGoal() {
        return Math.sqrt(Math.pow(targetPose.getX() - follower.getPose().getX(), 2) + Math.pow(targetPose.getY() - follower.getPose().getY(), 2));
    }
    public boolean isAligned() {
        return Math.abs(follower.getHeading() - targetHeading) < 0.1;
    }
}