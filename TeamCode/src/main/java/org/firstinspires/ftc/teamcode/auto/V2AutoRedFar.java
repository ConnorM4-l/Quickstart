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

@Autonomous(name = "red auto far")
public class V2AutoRedFar extends OpMode {
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private DcMotorSimple intake = null;

    private Pose startPose = new Pose(88, 8, Math.toRadians(90));

    private int pathState = 0;

    private Outtake shotController;

    private Intake intakeController;

    private Timer autoTimer;

    private RedPaths bluePaths;

    private Follower follower;


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        bluePaths = new RedPaths(follower);
        follower.setStartingPose(startPose);


        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        intake = hardwareMap.get(DcMotorSimple.class, "intakeMotor");

        intakeController = new Intake(hardwareMap);
        shotController = new Outtake(hardwareMap);

        autoTimer = new Timer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        shotController.update(1650);

        telemetry.addData("path state", pathState);
        telemetry.addData("launching state", shotController.launchingState());
        telemetry.addData("balls in robot", shotController.getBallsInRobot());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("left launcher error", shotController.getLeftErr());
        telemetry.addData("right launcher error", shotController.getRightErr());
        telemetry.addData("left launcher velocity", shotController.getLeftVelocity());
        telemetry.addData("right launcher velocity", shotController.getRightVelocity());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(bluePaths.leaveStartToShotLocation, 0.5, true);
                autoTimer.resetTimer();
                pathState = 1;
                break;
            case 1:
                if (!follower.isBusy()) {
                    //shoot here
                    if (shotController.shootTwoThenOne()) { //this tells you if the shot is done
                        follower.followPath(bluePaths.goIntakeOneBall);
                        //spin intake
                        autoTimer.resetTimer();
                        pathState = 2;
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(bluePaths.moveBack);
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(bluePaths.goIntakeTwoBalls);
                    pathState = 4;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(bluePaths.goBackToShotLocation);
                    intakeController.spin(0);
                    pathState = 5;
                }
                break;
            case 5:
                //configure the 10 inches to the max distance away from which you can start shooting
                if (follower.getDistanceRemaining() < 10) {
                    if (shotController.shootTwoThenOne()) {
                        //shoot
                        follower.followPath(bluePaths.goIntakeFirstInRow);
                        intakeController.spin(1);
                        pathState = 6;
                    }
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(bluePaths.goIntakeRestInRow);
                    pathState = 7;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    intakeController.spin(0);
                    pathState = 8;
                }
                break;
            case 8:
                follower.followPath(bluePaths.moveBackToShotThirdTime);
                pathState = 9;
                intakeController.spin(0);
                break;

            case 9:
                if (follower.getDistanceRemaining() < 10) {
                    if (shotController.shootTwoThenOne()) {
                        //shoot
                        follower.followPath(bluePaths.goToPark);
                        pathState = 10;
                    }
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    pathState = -1;
                }
                break;
        }
    }
}
