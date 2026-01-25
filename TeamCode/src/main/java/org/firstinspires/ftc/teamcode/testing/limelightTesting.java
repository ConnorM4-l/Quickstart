package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.limelight;

@TeleOp (name = "Limelight Testing")
public class limelightTesting extends OpMode {
    private limelight llController;

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        llController = new limelight(hardwareMap, true);
    }

    @Override
    public void start() {
        llController.start();
    }

    @Override
    public void loop() {
        llController.update();

        telemetry.addData("Tag ID", llController.getID());
        telemetry.addData("Valid", llController.getValid());
        telemetry.update();
    }

}
