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
    private Limelight3A limelight = null;
    private limelight visionController;

    private TelemetryManager telemetryM;

    private boolean automatedDrive = false;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(67, 67, Math.toRadians(45)));

        visionController = new limelight(hardwareMap, true);
    }

    @Override
    public void start() {
        limelight.start();
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        visionController.update();


        if (automatedDrive) {
            follower.turnToDegrees(visionController.getDesiredHeading());
        }
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true // Robot Centric (false for field centric)
        );
        if (gamepad1.rightBumperWasPressed()) {
            automatedDrive = true;
        } else if (gamepad1.leftBumperWasPressed()) {
            automatedDrive = false;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}