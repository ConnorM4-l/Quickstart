package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class RobotContext {
    public static Pose lastPose = new Pose (9,9,Math.toRadians(90));

    public RobotContext() {
        //utility class
    }

    public Pose getStartingPose() {
        return lastPose;
    }
}
