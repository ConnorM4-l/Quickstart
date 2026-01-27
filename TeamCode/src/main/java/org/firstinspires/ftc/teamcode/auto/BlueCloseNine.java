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
import org.firstinspires.ftc.teamcode.subsystem.colorDetector;


@Autonomous(name = "blue close nine")
public class BlueCloseNine extends OpMode {
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private DcMotorSimple intake = null;

    private Pose startPose;

    private int pathState = 0;

    private int motif = 0;

    private Outtake shotController;

    private Intake intakeController;

    private limelight llController;
    private Limelight3A limelight;

    private colorDetector ballColors;
    private boolean autoShotActive = false;
    private boolean autoShotSetupDone = false;



    private Timer autoTimer, globalTimer;
    private double globalTime;

    private BluePaths bluePaths;

    private Follower follower;

    private double autoTime;

    private double requestedDistance = 100;

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

        ballColors = new colorDetector(hardwareMap);



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
        ballColors.update();

        shotController.update(requestedDistance);
        autonomousPathUpdate();


        telemetry.addData("path state", pathState);
        telemetry.addData("launching state", shotController.launchingState());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Tag ID", llController.getID());
        telemetry.addData("Valid", llController.getValid());
        telemetry.addData("motif", motif);
        telemetry.addData("autoShotActive", autoShotActive);
        telemetry.addData("autoShotSetupDone", autoShotSetupDone);
        telemetry.addData("greenPos", ballColors.getGreenPosition()); // only if you have getter; otherwise print local
        telemetry.addData("leftColor", ballColors.detectLeftBallColor());
        telemetry.addData("rightColor", ballColors.detectRightBallColor());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(bluePaths.shoot1Path);
                autoTimer.resetTimer();
                pathState = 1;
                requestedDistance = 100;
                break;
            case 1:
                // Keep grabbing motif until we have it
                if (motif == 0) {
                    llController.update();
                    if (llController.getValid()) {
                        motif = llController.getID();          // 21/22/23
                        shotController.setMotif(motif);
                        llController.stop();
                    }
                }

                // When we arrive at the shooting spot, start + run the ordered shooting routine
                if (!follower.isBusy()) {
                    // Start only once
                    if (!autoShotSetupDone) {
                        startAutoShootOnce();
                    }

                    // Run until finished
                    runAutoShootUntilDone();

                    // When finished, leave to intake path
                    if (!autoShotActive) {
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
                    requestedDistance = 110;
                    pathState = 3;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    if (!autoShotSetupDone) {
                        startAutoShootOnce();
                    }

                    runAutoShootUntilDone();

                    if (!autoShotActive) {
                        follower.followPath(bluePaths.intake2Path);
                        intakeController.spin(1);
                        autoTimer.resetTimer();
                        pathState = 4;  // go to next path state after intake2 starts
                    }
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(bluePaths.shoot3Path);
                    autoTimer.resetTimer();
                    requestedDistance = 115;
                    pathState = 5;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    if (!autoShotSetupDone) {
                        startAutoShootOnce();
                    }

                    runAutoShootUntilDone();

                    if (!autoShotActive) {
                        pathState = -1;
                    }
                }
                break;

        }
    }

    private void startAutoShootOnce() {
        // run once per shooting sequence

        int positionGreen = ballColors.getGreenPosition(); // 1 left, 2 right, 3 back, 0 unknown

        shotController.setPositionGreen(positionGreen);

        shotController.setMotif(motif);

        autoShotActive = true;
        autoShotSetupDone = true;
    }

    private void runAutoShootUntilDone() {
        if (!autoShotActive) return;

        shotController.shootOrdered();

        if (!shotController.isStillShooting()) {
            autoShotActive = false;
            autoShotSetupDone = false;
            shotController.noShoot();
        }
    }

}
