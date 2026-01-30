package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import static java.lang.Thread.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.logitechCamera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Disabled
@Autonomous
public class V2AutoBlue extends OpMode {
    PathChain Path1;
    PathChain Path2;
    PathChain Path3;
    PathChain Path4;
    PathChain Path5;

    private Timer pathTimer;
    private Timer opmodeTimer;
    private logitechCamera cam;
    private int tagID;

    private Pose startPose = new Pose(33.61467889908257, 134.60550458715596, Math.toRadians(90));

    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private DcMotorSimple intake = null;

    private Outtake shotController;
    private Intake intakeController;


    @Override
    public void init() {
        tagID = -1;
        cam = new logitechCamera(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        intake = hardwareMap.get(DcMotorSimple.class, "intakeMotor");

        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeController = new Intake(hardwareMap);
        shotController = new Outtake(hardwareMap);
    }

    @Override
    public void loop() {
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        shotController.update(1250);

        telemetry.addData("path state", pathState);
        telemetry.update();
    }

    public void buildPaths() {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(33.615, 134.606), new Pose(63.266, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(63.266, 84.000), new Pose(40.807, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(40.807, 84.000), new Pose(35.817, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(35.817, 84.000), new Pose(16.000, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(16.000, 84.000), new Pose(63.266, 84.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                .build();
        }
        int pathState = 0;

        public void autonomousPathUpdate() throws InterruptedException {
            switch (pathState) {
                case 0:
                    follower.followPath(Path1);
                    pathTimer.resetTimer();
                    pathState = 1;
                    break;
                case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */      if (tagID == -1) {
                        cam.update();
                        for (AprilTagDetection a : cam.getDetectedTags())
                            if (a != null && (a.id >= 21 && a.id <= 23)) {
                                tagID = a.id;
                                break;
                            }
                    }
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    if (shotController.isStillShooting()) {
                        shotController.LRRShoot(true);
                    } else {
                        /* Score Preload */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        follower.followPath(Path2, true);
                        setPathState(2);
                    }
                    break;
                case 2:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                    if(!follower.isBusy()) {
                        /* Grab Sample */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                        intakeController.spin(1);
                        follower.followPath(Path3, 0.5, true);
                        setPathState(3);
                    }
                    break;
                case 3:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    if(!follower.isBusy()) {
                        /* Score Sample */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                        sleep(200);
                        follower.followPath(Path4, 0.5, true);
                        intakeController.spin(0);
                        setPathState(4);
                    }
                    break;
                case 4:
                    /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                    if(!follower.isBusy()) {
                        /* Grab Sample */

                        /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                        follower.followPath(Path5,true);
                        setPathState(5);
                    }
                    break;
            }
        }

        /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
        public void setPathState(int pState) {
            pathState = pState;
            pathTimer.resetTimer();
        }
    }
