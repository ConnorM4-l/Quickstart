package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.localization.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
@TeleOp(name = "Launcher Testing")
public class LauncherTesting extends OpMode {
    private PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    TelemetryManager telemetryM = panelsTelemetry.getTelemetry();

    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private TripleShot shotController;

    private boolean shotPressed = false;
    public static double launcherTargetVelocity = 3500;
    public static boolean launch = false;
    private int hasShot = 500;

    @Override
    public void init() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shotController = new TripleShot(hardwareMap);
    }

    @Override
    public void loop() {
        shotController.update(launch, gamepad1.left_bumper, launcherTargetVelocity);

        if (shotController.getHasShot()) {
            hasShot = 1000;
        } else {
            hasShot = 500;
        }

        telemetryM.addData("Count Shots Fired", shotController.getCountShots());
        telemetryM.addData("Error", shotController.getErr());
        telemetryM.addData("Launch State", shotController.getLaunchingState());
        telemetryM.addData("Launcher target velocity", launcherTargetVelocity);
        telemetryM.addData("Launcher velocity", shotController.getVelocity());
        telemetryM.addData("Launcher Acceleration", shotController.getAcceleration());
        telemetryM.addData("Has shot?", hasShot);
        telemetryM.update();
    }
}