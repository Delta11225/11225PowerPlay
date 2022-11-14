package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {

//        TrajectorySequenceFactory traj = new TrajectorySequenceFactory(new Pose2d(0, 0, 0));
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13.25, 12.25)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(57.5 * 0.9, 30, 5.398889 * 0.9, Math.toRadians(180), 12.12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39, 70-(12.25/2.0), Math.toRadians(270)))
////                                // Right parking (3)
//                                .strafeRight(35-16)
//                                .splineToConstantHeading(new Vector2d(-59, 36), Math.toRadians(270))
                                .setTurnConstraint(30, 2)
                                .splineToLinearHeading(new Pose2d(-28, 53.8, Math.toRadians(300)), Math.toRadians(300))
                                .splineToLinearHeading(new Pose2d(-27.7, 58.5, Math.toRadians(270)), Math.toRadians(270))
                                .strafeTo(new Vector2d(-14.2, 58.5))
                                .splineToConstantHeading(new Vector2d(-12.2, 58.5), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(-12.2, 12), Math.toRadians(270))
                                .turn(Math.toRadians(-90))
//                                .splineTo(new Vector2d(-14.2, 10), Math.toRadians(180))
                                .addDisplacementMarker(() -> {})
                                .splineToConstantHeading(new Vector2d(-59, 12), Math.toRadians(180))
                                .addDisplacementMarker(() -> {})
                                .waitSeconds(0.75)
                                .addDisplacementMarker(() -> {})
                                .back(10)
                                .splineToLinearHeading(new Pose2d(-52, 17.6, Math.toRadians(45)), Math.toRadians(45))
                                .addDisplacementMarker(() -> {})
                                .splineToLinearHeading(new Pose2d(-56, 12.5, Math.toRadians(90)), Math.toRadians(90))
                                // parking 3
//                                .forward(20)
                                // parking 2
//                                .strafeTo(new Vector2d(-36, 12.5))
//                                .splineToConstantHeading(new Vector2d(-34.5, 14), Math.toRadians(90))
//                                .splineTo(new Vector2d(-34.5, 34), Math.toRadians(90))
//                              // parking 1
//                                .strafeTo(new Vector2d(-14, 12.5))
//                                .splineToConstantHeading(new Vector2d(-12.2, 14), Math.toRadians(90))
//                                .splineTo(new Vector2d(-12.2, 34), Math.toRadians(90))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}