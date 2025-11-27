package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystem.TripleShot;
import org.firstinspires.ftc.teamcode.subsystem.flywheel;

@Configurable
@TeleOp(name = "Launcher Testing")
public class LauncherTesting extends OpMode {
    private PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    TelemetryManager telemetryM = panelsTelemetry.getTelemetry();

    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private flywheel flywheelController;

    private boolean shotPressed = false;
    public static double launcherTargetVelocity = 3500;
    public static boolean launch = false;
    public static boolean stop = false;
    private int hasShot = 500;

    @Override
    public void init() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelController = new flywheel(hardwareMap);
    }

    @Override
    public void loop() {
        flywheelController.update(launcherTargetVelocity);

        telemetryM.addData("Error", flywheelController.getErr());
        telemetryM.addData("Launcher target velocity", launcherTargetVelocity);
        telemetryM.addData("Launcher velocity", flywheelController.getVelocity());
        telemetryM.addData("Launcher Acceleration", flywheelController.getAcceleration());
        telemetryM.update();
    }
}