package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Coordinate;

@TeleOp(name = "SecondTeleOp")
public class SecondCompTeleOp extends OpMode {
    private DcMotor leftOdo;
    private DcMotor rightOdo;
    private DcMotor strafeOdo;
    private IMU imu;
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
    private double launcherVelocity = 3500;

    private Boolean faceTowerStrafe = false;
    double curOrientation;
    private RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot( RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    private Coordinate tower = new Coordinate();


    @Override
    public void init() {
        bl = hardwareMap.get(DcMotor.class, "bl");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        leftOdo = hardwareMap.get(DcMotor.class, "leftOdo");
        rightOdo = hardwareMap.get(DcMotor.class, "rightOdo");
        strafeOdo = hardwareMap.get(DcMotor.class, "strafeOdo");
        imu = hardwareMap.get(IMU.class, "imu");

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
        movementController = new Drivetrain(hardwareMap, new Coordinate(0.0,0.0), 0.0);


    }
    public double getAngle() {return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);}

    @Override
    public void loop() {
        curOrientation = getAngle();
        double rotationVal = imu.getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate; // might not be xRotation rate need to test
        shotController.update(gamepad1.right_bumper, gamepad1.left_bumper, launcherVelocity);

        if (faceTowerStrafe)
        {
            movementController.updateFacingPoint(-gamepad1.left_stick_y, gamepad1.left_stick_x, tower, curOrientation, rotationVal);
        } else
            movementController.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);


        telemetry.addData("Error", shotController.getErr());
        telemetry.addData("Launch State", shotController.getLaunchingState());
        telemetry.addData("Heading",getAngle());
        telemetry.addData("Rotation Rate",rotationVal);
        telemetry.update();
    }
}
