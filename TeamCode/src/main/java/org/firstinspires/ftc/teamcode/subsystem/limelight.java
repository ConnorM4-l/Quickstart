package org.firstinspires.ftc.teamcode.subsystem;

import android.view.textclassifier.TextClassifierEvent;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Coordinate;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class limelight {
    private Limelight3A limelight = null;
    private double x;
    private double y;
    private double heading;
    private boolean valid = false;
    private LLResult llResult;
    private List<LLResultTypes.FiducialResult> frResult;
    private AprilTagProcessor aprilTagProcessor;

    //alliance should be true for blue
    private boolean alliance;

    public limelight(HardwareMap hardwareMap, boolean all) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);

        alliance = all;
    }

    public void start() {
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    public void stop() {
        limelight.stop();
    }


    public void update() {
        LLResult latestResult = limelight.getLatestResult();

        if (latestResult != null && latestResult.isValid()) {
            llResult = latestResult;
            frResult = llResult.getFiducialResults();
            valid = true;
        } else {
            valid = false;
        }


    }
    public Pose getPose() {
        return new Pose(x, y, heading);
    }

    public Coordinate getCoordinate() {
        return new Coordinate(x, y);
    }

    public double getHeading() {
        return heading;
    }

    public double getDistance() {
        return Math.sqrt(x * x + y * y);
    }

    public double getDesiredHeading() {
        return Math.toDegrees(Math.atan2(y + 67, x + 67));
    }

    Pose getCamPose() {
        if (llResult != null) {
            Pose3D botpose = limelight.getLatestResult().getBotpose_MT2();
            Pose3D botpose2 = limelight.getLatestResult().getBotpose();

            Position poseIn = botpose.getPosition().toUnit(DistanceUnit.INCH);

            Pose limelightPose = new Pose(poseIn.x, poseIn.y, botpose2.getOrientation().getYaw());
            Pose pedroPose = new Pose(limelightPose.getX(), limelightPose.getY(), limelightPose.getHeading(), FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);


            return pedroPose;
        } else {
            return new Pose();
        }
    }

    public int getID() {
        if (valid) {
            for (LLResultTypes.FiducialResult frRes : frResult) {
                int id = frRes.getFiducialId();
                if (id > 20 && id < 24) {
                    return id;
                }
            }
        }
        return 0;
    }

    public boolean getValid() {
        return valid;
    }
}
