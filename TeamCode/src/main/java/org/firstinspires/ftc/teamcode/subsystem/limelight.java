package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Coordinate;

public class limelight {
    private Limelight3A limelight = null;
    private double x;
    private double y;
    private double heading;

    //alliance should be true for blue
    private boolean alliance;

    public limelight(HardwareMap hardwareMap, boolean all) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.start();

        //0 means it uses april tags
        limelight.pipelineSwitch(0);

        alliance = all;
    }

    public void update() {
        LLResult latestResult = limelight.getLatestResult();


        // get the botpose (pose3d)
        Pose3D botpose = latestResult.getBotpose();

        //in meters
        Position p = botpose.getPosition();
        YawPitchRollAngles rpy = botpose.getOrientation();

        //new position in inches
        x = p.x * 39.3701;
        y = p.y * 39.3701;
        heading = rpy.getYaw(AngleUnit.RADIANS);

        //add these values to get the coordinates with respect to the goal
        if (alliance) {
            x -= 67;
            y += 67;
        } else {
            x -= 67;
            y -= 67;
        }

        if (!latestResult.isValid()) {
            x = 0;
            y = 0;
            heading = 0;
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
        if (limelight.getLatestResult() != null) {
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
}
