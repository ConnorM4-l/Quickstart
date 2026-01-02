package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

@Autonomous(name = "blue auto far")
public class V2AutoBlueFar extends OpMode {

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;

    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private DcMotorSimple intake = null;

    private Pose startPose = new Pose(56, 8, Math.toRadians(90));

    private int pathState = 0;

    private Outtake shotController;

    private Intake intakeController;

    private Timer autoTimer;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        buildPaths(follower);
        follower.setStartingPose(startPose);

        shotController = new Outtake(hardwareMap);

        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        intake = hardwareMap.get(DcMotorSimple.class, "intakeMotor");

        intakeController = new Intake(hardwareMap);

        autoTimer = new Timer();
    }

    @Override
    public void loop() {
        autonomousPathUpdate();
        shotController.update(1650);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(Path1);
                pathState = 1;
                break;
            case 1:
                follower.followPath(Path2);
                autoTimer.resetTimer();
                pathState = 2;
                break;
            case 2:
                double autoTime = autoTimer.getElapsedTimeSeconds();
                if (autoTime < 3) {
                    shotController.shootLeft();
                } else if (autoTime < 9) {
                    shotController.shootRight();
                    intakeController.spin(1);
                } else if (autoTime > 9) {
                    intakeController.spin(0);
                    pathState = 3;
                }
                break;
            case 3:
            follower.followPath(Path3);
            pathState = -1;
            break;

        }
    }

    public void buildPaths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 7.927), new Pose(56.000, 13.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 13.500), new Pose(56.000, 13.500))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112))
                .build();
        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 13.500), new Pose(50.000, 24.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(112))
                .build();
    }
}
