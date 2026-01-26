package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.limelight;

@Autonomous(name = "blue close nine")
public class BlueCloseNine extends OpMode {
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private DcMotorSimple intake = null;

    private Pose startPose;

    private int pathState = 0;

    private int motif;

    private Outtake shotController;

    private Intake intakeController;

    private limelight llController;
    private Limelight3A limelight;


    private Timer autoTimer, globalTimer;
    private double globalTime;

    private BluePaths bluePaths;

    private Follower follower;

    private double autoTime;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        bluePaths = new BluePaths(AutoType.CLOSE_NINE, follower);

        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        intake = hardwareMap.get(DcMotorSimple.class, "intakeMotor");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        intakeController = new Intake(hardwareMap);
        shotController = new Outtake(hardwareMap);
        llController = new limelight(hardwareMap, true);


        autoTimer = new Timer();
        globalTimer = new Timer();
    }

    @Override
    public void start() {
        llController.start();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();


        telemetry.addData("path state", pathState);
        telemetry.addData("launching state", shotController.launchingState());
        telemetry.addData("balls in robot", shotController.getBallsInRobot());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Tag ID", llController.getID());
        telemetry.addData("Valid", llController.getValid());
        telemetry.addData("motif", motif);
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(bluePaths.shoot1Path);
                autoTimer.resetTimer();
                shotController.setPositionGreen(1);
                pathState = 1;
                break;
            case 1:

                if (motif == 0) {
                    llController.update();
                     if (llController.getValid()) {
                         motif = llController.getID();
                         shotController.setMotif(motif);
                         llController.stop();
                     }
                }
                if (!follower.isBusy()) {
                    autoTime = autoTimer.getElapsedTimeSeconds();
                    shotController.shootOrdered();
                    if (shotController.isStillShooting()) {
                        follower.followPath(bluePaths.intake1Path);
                        intakeController.spin(1);
                        autoTimer.resetTimer();
                        pathState = 2;
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(bluePaths.shoot2Path);
                    autoTimer.resetTimer();
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    autoTime = autoTimer.getElapsedTimeSeconds();
                    if (autoTime > 1) {
                        follower.followPath(bluePaths.intake2Path);
                        intakeController.spin(1);
                        autoTimer.resetTimer();
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(bluePaths.shoot3Path);
                    autoTimer.resetTimer();
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    autoTime = autoTimer.getElapsedTimeSeconds();
                    if (autoTime > 1) {
                        pathState = -1;
                    }
                }
                break;
        }
    }
}
