package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Spline Testing")
public class SplineTesting extends OpMode {
    private PanelsTelemetry panelsTelemetry = PanelsTelemetry.INSTANCE;
    private Pose startingPose = new Pose(0, 0, -0);
    TelemetryManager telemetryM = panelsTelemetry.getTelemetry();
    public static double DISTANCE = 40;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {
        telemetryM.debug("This will activate all the PIDF(s)");
        telemetryM.debug("The robot will go forward and backward continuously along the path while correcting.");
        telemetryM.debug("You can adjust the PIDF values to tune the robot's drive PIDF(s).");
        telemetryM.update(telemetry);
        follower.update();
        drawOnlyCurrent();
    }

    @Override
    public void start() {

        follower.activateAllPIDFs();
        forwards = new Path(new BezierCurve(
                new Pose(0, 0),
                new Pose(0, 24),
                new Pose(24, 48),
                new Pose(24, 72)
        ));
        forwards.setTangentHeadingInterpolation();
        backwards = new Path(new BezierCurve(
                new Pose(24, 72),
                new Pose(24, 48),
                new Pose(0, 24),
                new Pose(0, 0)));
                backwards.setTangentHeadingInterpolation();
        follower.followPath(forwards);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
    @Override
    public void loop() {
        follower.update();
        draw();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryM.debug("Driving Forward?: " + forward);
        telemetryM.update(telemetry);
    }
}
