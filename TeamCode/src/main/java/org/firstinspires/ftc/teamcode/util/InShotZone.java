package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class InShotZone {
    private Pose currentPose;

    public InShotZone() {
        currentPose = new Pose();
    }

    public void setCurrentPose(Pose pose) {
        currentPose = pose;
    }

    public boolean isInShotZone() {
        double x = currentPose.getX();
        double y = currentPose.getY();



    }

}
