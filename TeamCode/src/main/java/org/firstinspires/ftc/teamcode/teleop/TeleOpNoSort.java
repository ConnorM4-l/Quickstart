package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.limelight;
import org.firstinspires.ftc.teamcode.subsystem.drivetrain;
import org.firstinspires.ftc.teamcode.util.VelocitySolver;

@Disabled
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


    private Outtake shotController;
    private drivetrain movementController;
    private limelight visionController;
    private VelocitySolver velocitySolver;

    private double launcherInitVelocity = 2700;

    private double desiredHeading = 3.14 / 2;

    private boolean following = false;
    private final Pose TARGET_LOCATION = new Pose(67, 67, 45);

    @Override
    public void init() {
        bl = hardwareMap.get(DcMotor.class, "bl");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
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

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());

        shotController = new Outtake(hardwareMap);
        movementController = new drivetrain(hardwareMap);
        visionController = new limelight(hardwareMap, true);
    }
    @Override
    public void loop() {
        follower.update();
        visionController.update();

        desiredHeading = visionController.getDesiredHeading();

//        if (gamepad1.rightBumperWasPressed()) {
//            if (movementController.isFacingTower(visionController.getCoordinate())) {
//                shotController.update(velocitySolver.getVelocity(visionController.getDistance()), 1, 0.5);
//            } else {
//                shotController.update(velocitySolver.getVelocity(visionController.getDistance()), 1, 0.5);
//            }
//            if (!following) {
////                follower.followPath(
////                        follower.pathBuilder()
////                                .addPath(new BezierLine(follower.getPose(), TARGET_LOCATION))
////                                .setLinearHeadingInterpolation(follower.getHeading(), TARGET_LOCATION.minus(follower.getPose()).getAsVector().getTheta())
////                                .build()
////                );
//                follower.turnToDegrees(visionController.getDesiredHeading());
//            }
//        } else {
//            movementController.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//            shotController.update(launcherInitVelocity, 1, 0.5);
//        }
        follower.setPose(getRobotPoseFromCamera());

        if (following && !follower.isBusy()) following = false;
    }

    private Pose getRobotPoseFromCamera() {
        //Fill this out to get the robot Pose from the camera's output (apply any filters if you need to using follower.getPose() for fusion)
        //Pedro Pathing has built-in KalmanFilter and LowPassFilter classes you can use for this
        //Use this to convert standard FTC coordinates to standard Pedro Pathing coordinates
        return new Pose(0, 0, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
    }
}
