package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {

//        TrajectorySequenceFactory traj = new TrajectorySequenceFactory(new Pose2d(0, 0, 0));
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d startPose = new Pose2d(29.5, 70 - (12.25 / 2.0), Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13.25, 12.25)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(57.5 * 0.9, 30, 5.398889 * 0.9, Math.toRadians(180), 12.12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .strafeTo(new Vector2d(-6, 60))
                                .splineToLinearHeading(new Pose2d(-6, 33, Math.toRadians(90)), Math.toRadians(270))
                                .strafeTo(new Vector2d(-40, 33)
//                                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .waitSeconds(0.25)
                                .strafeTo(new Vector2d(-32, 33))
        // .splineToLinearHeading(new Pose2d(-32, , Math.toRadians(90)), Math.toRadians(90));
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}