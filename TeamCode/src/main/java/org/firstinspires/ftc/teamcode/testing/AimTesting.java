package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.limelight;

@TeleOp(name = "AimTesting")
public class AimTesting extends OpMode {

    private TelemetryManager telemetryM;

    private boolean automatedDrive = false;

    private Pose targetPose = new Pose(-67, -67, 0);

    private double targetHeading = 0;


    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(67, 67, Math.toRadians(45)));
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

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

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("is aligned", isAligned());
        telemetryM.debug("distance from goal", distanceFromGoal());
        telemetryM.debug("target heading", targetHeading);
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