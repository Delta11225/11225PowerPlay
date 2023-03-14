package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRedFront {
    public static void main(String[] args) {

//        TrajectorySequenceFactory traj = new TrajectorySequenceFactory(new Pose2d(0, 0, 0));
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13.25, 12.25)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(57.5 * 0.9, 30, 5.398889 * 0.9, Math.toRadians(180), 12.12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-40, -70 + (12.25 / 2.0), Math.toRadians(90)))
//                            Pose2d startPose = new Pose2d(-40, 70 - (12.25 / 2.0), Math.toRadians(270));
              .setTurnConstraint(60, 5)
              .addDisplacementMarker(() -> {

              })
              .strafeTo(new Vector2d(-12.5, -65))
              .splineToConstantHeading(new Vector2d(-10, -57.1), Math.toRadians(90))
              .addDisplacementMarker(() -> {

              })
              .splineToLinearHeading(new Pose2d(-5.6, -31, Math.toRadians(60)), Math.toRadians(60))
              .addDisplacementMarker(() -> {

              })

              // Back up
              .splineToLinearHeading(new Pose2d(-13, -30, Math.toRadians(90)), Math.toRadians(90))

              // Go to turn
              .splineToLinearHeading(new Pose2d(-16, -12, Math.toRadians(180)), Math.toRadians(180))
              .addDisplacementMarker(() -> {

              })
              .splineToConstantHeading(new Vector2d(-60, -12), Math.toRadians(180))
              .waitSeconds(0.25)
              .addDisplacementMarker(() -> {
              })
              // Do not ever do this. This is a hack.
              .forward(0.001)
              .waitSeconds(0.25)
              .addDisplacementMarker(() -> {
              })
              .back(20)
              .addDisplacementMarker(() -> {
              })
              .splineToLinearHeading(new Pose2d(-28.5, -8.5, Math.toRadians(60)), Math.toRadians(60))
              .addDisplacementMarker(() -> {
              })
              .back(10)
              .addDisplacementMarker(() -> {
              })
              .forward(0.001)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}