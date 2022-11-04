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
                                .splineToLinearHeading(new Pose2d(-28, 53.8, Math.toRadians(300)), Math.toRadians(300))
                                .splineToLinearHeading(new Pose2d(-27.7, 58.5, Math.toRadians(270)), Math.toRadians(270))
                                .strafeTo(new Vector2d(-14.2, 58.5))
                                .splineToConstantHeading(new Vector2d(-12.2, 58.5), Math.toRadians(270))
                                .strafeTo(new Vector2d(-12.2, 10))
                                .splineToLinearHeading(new Pose2d(-14.2, 10, Math.toRadians(180)), Math.toRadians(180))

//
//                                  // Middle parking (2)
////                                .forward(70-(12.25/2.0) - 36)
//
//                                  // Left parking (1)
//                                .strafeLeft(35-16)
//                                .splineToConstantHeading(new Vector2d(59, 36), Math.toRadians(270))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}