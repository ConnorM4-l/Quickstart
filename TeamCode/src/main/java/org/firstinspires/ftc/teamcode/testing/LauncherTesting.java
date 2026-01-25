package org.firstinspires.ftc.teamcode.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain;
import org.firstinspires.ftc.teamcode.util.VelocitySolver;

@Configurable
@TeleOp(name = "Launcher Testing")
public class LauncherTesting extends OpMode {
    private PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    TelemetryManager telemetryM = panelsTelemetry.getTelemetry();

    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;

    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private Outtake shotController;
    private VelocitySolver velocitySolver;

    public static double launcherInitVelocity = 2700;


    public static boolean shootLeft = false;
    public static boolean shootRight = false;
    public static boolean shootBoth = false;

    @Override
    public void init() {
        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);

        shotController = new Outtake(hardwareMap);
        //movementController = new drivetrain(hardwareMap, location);
        //visionController = new limelight(hardwareMap, true);
    }

    @Override
    public void loop() {
        //shotController.update(distanceVelocityMultiple * visionController.getDistance(), true, 1, 0.5);
        shotController.update(launcherInitVelocity);

        if (shootBoth) {
            shotController.shootBoth();
        }

        //1250 for corner


        telemetryM.addData("Left error", shotController.getLeftErr());
        telemetryM.addData("right error", shotController.getRightErr());
        //telemetryM.addData("Launcher target velocity", velocitySolver.getVelocity(visionController.getDistance()));
        telemetryM.addData("Launcher target velocity", launcherInitVelocity);
        telemetryM.addData("Launcher left velocity", shotController.getLeftVelocity());
        telemetryM.addData("Launcher right velocity", shotController.getRightVelocity());
        telemetryM.addData("Launcher acceleration", shotController.getLeftAcceleration());

        telemetryM.update();
    }
}