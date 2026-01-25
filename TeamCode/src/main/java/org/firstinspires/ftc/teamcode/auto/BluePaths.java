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

    public Pose startPose, shoot1, intake1, shoot2, intake2, shoot3, intake3, shoot4, intake4, shoot5, intake5, shoot6, intake6, control1, control2, control3, control4, control5, control6;

    public PathChain shoot1Path, shoot2Path, shoot3Path, shoot4Path, shoot5Path, shoot6Path, intake1Path, intake2Path, intake3Path, intake4Path, intake5Path, intake6Path;

    public BluePaths (AutoType a, Follower f) {
        follower = f;
        autoType = a;
    }

    public void buildPaths() {
        if (autoType == AutoType.FAR_SIX) {
            buildFarSix();
        } else if (autoType == AutoType.CLOSE_NINE) {
            buildCloseNine();
        } else if (autoType == AutoType.CLOSE_TWELVE) {
            //buildCloseTwelve();
        }
    }

    public void buildFarSix() {
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

    public void buildCloseNine() {
        startPose = new Pose(21.055, 121.679);
        control1 = new Pose(32.454, 113.225);
        control2 = new Pose(35.683, 101.041);
        shoot1 = new Pose(46.312, 88.404);
        control3 = new Pose(38.578, 84.041);
        intake1 = new Pose(20.789, 84.156);
        shoot2 = new Pose(51.725, 84.009);
        control4 = new Pose(42.390, 60.174);
        intake2 = new Pose(22.807, 59.807);
        shoot3 = new Pose(57.229, 104.642);

        int angle1 = 180;
        int angle2 = 130;
        int angle3 = 180;
        int angle4 = 140;

        shoot1Path = follower.pathBuilder().addPath(
                        new BezierCurve(
                                startPose, control1, control2, shoot1
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
        intake1Path = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shoot1, control3, intake1
                        )
                ).setTangentHeadingInterpolation()
                .build();
        shoot2Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                intake1, shoot2
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(angle1), Math.toRadians(angle2))
                .build();
        intake2Path = follower.pathBuilder().addPath(
                        new BezierCurve(
                                shoot2, control4, intake2
                        )
                ).setTangentHeadingInterpolation()
                .build();
        shoot3Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                intake2, shoot3
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(angle3), Math.toRadians(angle4))
                .build();
    }
}