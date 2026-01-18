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
    public PathChain leaveShotFinal;
    public PathChain goIntakeFirstInRow;
    public PathChain goIntakeRestInRow;
    public PathChain moveBackToShotThirdTime;
    public PathChain goToPark;

    //make an enum that stores the different paths like closeSixBall, farSixBall, ect.
    public BluePaths(Follower follower) {
        leaveStartToShotLocation = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 8.000), new Pose(56.073, 15.706))
                )
                //reference value was 25, but 0.5 was very slow
                //.setVelocityConstraint(0.5)
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(112))
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
                .setLinearHeadingInterpolation(Math.toRadians(8), Math.toRadians(29))
                .setReversed()
                .build();

        goIntakeTwoBalls = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(31.266, 16.147),
                                new Pose(19.669, 9.688),
                                new Pose(8.514, 9.661)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        goBackToShotLocation = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(8.514, 8.661), new Pose(56.073, 15.706)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))
                .build();
        goIntakeFirstInRow = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.073, 15.706),

                                new Pose(34.422, 35.119)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(180))
                .build();

        goIntakeRestInRow = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(34.422, 35.119),

                                new Pose(9.037, 35.119)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        moveBackToShotThirdTime = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.037, 35.119),
                                new Pose(56.073, 15.706)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(112))
                .build();

        goToPark = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.073, 15.706),
                                new Pose(48.229, 61.954)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(180))
                .build();
    }
}