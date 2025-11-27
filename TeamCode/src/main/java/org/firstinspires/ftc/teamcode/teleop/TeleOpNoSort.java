package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.ftc.localization.Encoder;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.util.Coordinate;
import org.firstinspires.ftc.teamcode.ShootNoSort;
import org.firstinspires.ftc.teamcode.subsystem.limelight;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain;
import org.firstinspires.ftc.teamcode.util.VelocitySolver;

@TeleOp(name = "TeleOpNoSort")
public class TeleOpNoSort extends OpMode {
    private DcMotor bl = null;
    private DcMotor fl = null;
    private DcMotor br = null;
    private DcMotor fr = null;
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private Limelight3A limelight = null;
    private IMU imu = null;
    private Encoder leftOdom = null;
    private Encoder rightodom = null;
    private Encoder strafeOdom = null;


    private ShootNoSort shotController;
    private drivetrain movementController;
    private limelight visionController;
    private VelocitySolver velocitySolver;

    private double launcherInitVelocity = 2700;

    private Coordinate location = new Coordinate(67.0, 67.0);

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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        leftOdom = hardwareMap.get(DcMotor.class, "fl");
        rightodom = hardwareMap.get(DcMotor.class, "fr");
        strafeOdom = hardwareMap.get(DcMotor.class, "br");




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

        shotController = new ShootNoSort(hardwareMap);
        movementController = new drivetrain(hardwareMap, location);
        visionController = new limelight(hardwareMap, true);
    }
    @Override
    public void loop() {
        visionController.update();
        movementController.updateLocation(leftOdom.getDistance);

        if (gamepad1.rightBumperWasPressed()) {
            if (movementController.isFacingTower(visionController.getCoordinate())) {
                shotController.update(velocitySolver.getVelocity(visionController.getDistance()), true, 1, 0.5);
            } else {
                shotController.update(velocitySolver.getVelocity(visionController.getDistance()), false, 1, 0.5);
            }
            movementController.updateFacingPoint(-gamepad1.left_stick_y, gamepad1.left_stick_x, visionController.getCoordinate(), visionController.getHeading(), 0);
        } else {
            movementController.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            shotController.update(launcherInitVelocity, false, 1, 0.5);
        }
    }


}
