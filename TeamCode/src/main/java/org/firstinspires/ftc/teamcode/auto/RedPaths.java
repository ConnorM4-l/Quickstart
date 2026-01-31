package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

public class RedPaths {

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

    private BluePaths bluePaths = new BluePaths(AutoType.CLOSE_NINE, follower);

    //make an enum that stores the different paths like closeSixBall, farSixBall, ect.
    public RedPaths(Follower follower) {
        leaveStartToShotLocation = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88, 8.000), new Pose(87.9, 15.706))
                )
                //reference value was 25, but 0.5 was very slow
                //.setVelocityConstraint(0.5)
                //check angle of 180, not sure about it
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(68))
                .build();
        goIntakeOneBall = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(87.927, 15.706),
                                new Pose(109.44962790697676, 22.19767441860464),
                                new Pose(133.578, 18.1)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        moveBack = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144 - 10.422, 18.642), new Pose(144 - 31.266, 16.147))
                )
                .setLinearHeadingInterpolation(Math.toRadians(172), Math.toRadians(151))
                .setReversed()
                .build();

        goIntakeTwoBalls = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144 - 31.266, 16.147),
                                new Pose(144 - 14.385128440366975, 9.687733944954134),
                                new Pose(144 - 8.514, 9.661)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        goBackToShotLocation = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(144 - 8.514, 8.661), new Pose(144 - 56.073, 15.706)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                .build();
        goIntakeFirstInRow = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(144 - 56.073, 15.706),

                                new Pose(144 - 34.422, 35.119)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .build();

        goIntakeRestInRow = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(144 - 34.422, 35.119),

                                new Pose(144 - 9.037, 35.119)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        moveBackToShotThirdTime = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(144 - 9.037, 35.119),
                                new Pose(144 - 56.073, 15.706)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(68))
                .build();

        goToPark = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(144 - 56.073, 15.706),
                                new Pose(144 - 48.229, 61.954)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(68), Math.toRadians(0))
                .build();

    }
}
