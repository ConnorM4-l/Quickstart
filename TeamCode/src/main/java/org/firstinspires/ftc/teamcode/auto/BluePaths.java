package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

public class BluePaths {

    public final AutoType autoType;
    private final Follower follower;

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

    public Pose startPose, shoot1, intake1, shoot2, intake2, shoot3, intake3, shoot4, intake4, shoot5, intake5, shoot6, intake6, control1, control2, control3, control5, control4, control6, intake1_1, intake1_2, intake2_1, intake2_2, intake2_3;

    public PathChain shoot1Path, shoot2Path, shoot3Path, shoot4Path, shoot5Path, shoot6Path, intake1Path, intake2Path, intake3Path, intake4Path, intake5Path, intake6Path, intake1p1Path, intake1p2Path, intake1p3Path, intake2_1Path, intake2_2Path, intake2_3Path;

    public BluePaths (AutoType a, Follower f) {
        follower = f;
        autoType = a;
        buildPaths();
    }

    public void buildPaths() {
        if (autoType == AutoType.FAR_SIX) {
            buildFarSix();
        } else if (autoType == AutoType.CLOSE_NINE) {
            buildCloseNine();
        } else if (autoType == AutoType.CLOSE_TWELVE) {
            //buildCloseTwelve();
        } else if (autoType == AutoType.FAR_NINE) {
            buildFarNine();
        }
    }

    public void buildCloseNine() {
        shoot1Path = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(19.147, 119.477),
                                new Pose(29.225, 112.197),
                                new Pose(37.591, 102.656),
                                new Pose(47.633, 89.138)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intake1p1Path = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(47.633, 89.138),
                                new Pose(42.244, 83.888),
                                new Pose(16.714, 83.413)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        intake1p2Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.714, 83.413),

                                new Pose(22.183, 83.417)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intake1p3Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(22.183, 83.417),

                                new Pose(16.844, 83.273)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        shoot2Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.844, 83.273),

                                new Pose(51.725, 84.009)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(127.2))

                .build();

        intake2Path = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51.725, 84.009),
                                new Pose(43.936, 57.372),
                                new Pose(11.725, 57.523)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        shoot3Path = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(11.725, 57.523),
                                new Pose(35.867, 53.229),
                                new Pose(53.514, 110.789)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(149.2))

                .build();
    }

    private void buildFarSix() {
        shoot1Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 8.000),

                                new Pose(55.853, 16.330)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))

                .build();

        intake1Path = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.853, 16.330),
                                new Pose(51.216, 34.959),
                                new Pose(10.963, 35.367)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        shoot2Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.963, 35.367),

                                new Pose(55.404, 16.376)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))

                .build();

        intake2Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.404, 16.376),

                                new Pose(52.229, 33.807)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }

    private void buildFarNine() {
        shoot1Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 8.000),

                                new Pose(55.853, 16.330)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))

                .build();

        intake1Path = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.853, 16.330),
                                new Pose(51.216, 34.959),
                                new Pose(10.963, 35.367)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        shoot2Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.963, 35.367),

                                new Pose(55.853, 16.330)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))

                .build();

        intake2Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.853, 16.330),

                                new Pose(11.303, 13.211)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        shoot3Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.303, 13.211),

                                new Pose(55.853, 16.330)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(-176), Math.toRadians(110))

                .build();

        intake3Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.853, 16.330),

                                new Pose(52.000, 25.303)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
}