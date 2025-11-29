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

    private DcMotor bl = null;
    private DcMotor fl = null;
    private DcMotor br = null;
    private DcMotor fr = null;
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    //private Limelight3A limelight = null;
    private IMU imu = null;

    private Outtake shotController;
    private drivetrain movementController;
    //private limelight visionController;
    private VelocitySolver velocitySolver;

    public static double launcherInitVelocity = 2700;


    public static double distanceVelocityMultiple = 50;
    public static boolean shoot = false;

    public static double timeShot = 1;
    public static double timeBetween = 0.5;

    @Override
    public void init() {
        bl = hardwareMap.get(DcMotor.class, "bl");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);

        shotController = new Outtake(hardwareMap, timeShot, timeBetween);
        //movementController = new drivetrain(hardwareMap, location);
        //visionController = new limelight(hardwareMap, true);
    }

    @Override
    public void loop() {
        //shotController.update(distanceVelocityMultiple * visionController.getDistance(), true, 1, 0.5);
        shotController.update(launcherInitVelocity, shoot, 1, 0.5);



        telemetryM.addData("Error", shotController.getErr());
        //telemetryM.addData("Launcher target velocity", velocitySolver.getVelocity(visionController.getDistance()));
        telemetryM.addData("Launcher velocity", shotController.getVelocity());
        //telemetryM.addData("Launcher Acceleration", shotController.getAcceleration());
        telemetryM.addData("Heading", getAngle());
        telemetryM.update();
    }

    public double getAngle() {return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);}
}