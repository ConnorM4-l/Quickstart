package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

public class BluePaths {

    // -------- 6 ball far zone auto paths ---------

    public PathChain leaveStartToShotLocation;
    public PathChain goIntakeOneBall;
    public PathChain moveBack;
    public PathChain goIntakeTwoBalls;
    public PathChain goBackToShotLocation;

    public BluePaths(Follower follower) {
        leaveStartToShotLocation = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(56.073, 15.706))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114))
                .build();

        goIntakeOneBall = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(56.073, 15.706),
                                new Pose(26.862, 23.339),
                                new Pose(10.422, 18.642)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        moveBack = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(10.422, 18.642), new Pose(31.266, 16.147))
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        goIntakeTwoBalls = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(31.266, 16.147),
                                new Pose(21.431, 8.807),
                                new Pose(8.514, 8.661)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        goBackToShotLocation = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(8.514, 8.661), new Pose(56.073, 15.706)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(114))
                .build();
    }
}
