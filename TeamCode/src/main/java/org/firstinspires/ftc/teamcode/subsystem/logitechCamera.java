package org.firstinspires.ftc.teamcode.subsystem;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;


public class logitechCamera {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private ArrayList<AprilTagDetection> detectedTags = new ArrayList<AprilTagDetection>();

    public logitechCamera(WebcamName cam)
    {
        aprilTagProcessor = new AprilTagProcessor.Builder().setDrawTagID(true).setDrawTagOutline(true).setDrawAxes(true).setDrawCubeProjection(true).setOutputUnits(DistanceUnit.CM, AngleUnit.RADIANS).build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(cam);
        builder.setCameraResolution(new Size(640,480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update()
    {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public ArrayList<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }
    public AprilTagDetection getTagFromId(int id)
    {
        for( AprilTagDetection s : detectedTags)
            if(s.id == id)
                return s;
        return null;
    }
}
