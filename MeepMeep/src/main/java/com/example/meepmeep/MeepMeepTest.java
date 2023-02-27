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
        Pose2d startPose = new Pose2d(29.5, 70 - (12.25 / 2.0), Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13.25, 12.25)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(57.5 * 0.9, 30, 5.398889 * 0.9, Math.toRadians(180), 12.12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .addDisplacementMarker(() -> {
//                                    robot.rightClaw.setPosition(Constants.rightClawClosed);
//                                    robot.leftClaw.setPosition(Constants.leftClawClosed);
                                })
                                .strafeTo(new Vector2d(12.5, 60))
                                .splineToConstantHeading(new Vector2d(10, 57.1), Math.toRadians(270))
                                .addDisplacementMarker(() -> {
//                                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[2]);
//                                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                                    robot.linearSlide.setPower(1);
                                })
                                .splineToLinearHeading(new Pose2d(2, 31, Math.toRadians(240)), Math.toRadians(240))

                                .splineToLinearHeading(new Pose2d(0.5, 29.5, Math.toRadians(240)), Math.toRadians(240))
                                .addDisplacementMarker(() -> {
//                                    robot.leftClaw.setPosition(Constants.leftClawOpen);
//                                    robot.rightClaw.setPosition(Constants.rightClawOpen);
                                })
                                .waitSeconds(0.25)

                                // Back up
                                .splineToLinearHeading(new Pose2d(6, 36, Math.toRadians(240)), Math.toRadians(240))
                                .addDisplacementMarker(() -> {
//                                    robot.linearSlide.setTargetPosition(0);
//                                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                                    robot.linearSlide.setPower(1);
                                })

                                // square up
                                .splineToLinearHeading(new Pose2d(10, 30, Math.toRadians(270)), Math.toRadians(270))

                                // Go to turn
                                .splineToLinearHeading(new Pose2d(12, 8, Math.toRadians(0)), Math.toRadians(0))
                                .addDisplacementMarker(() -> {
//                                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderConeStack()[0]);
//                                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                                    robot.linearSlide.setPower(1);
                                })
                                //approach cone stack
                                .splineToConstantHeading(new Vector2d(56.5, 8), Math.toRadians(0))
                                .waitSeconds(0.25)

                                .addDisplacementMarker(() -> {
//                                    robot.rightClaw.setPosition(Constants.rightClawClosed);
//                                    robot.leftClaw.setPosition(Constants.leftClawClosed);
                                })
                                // Do not ever do this. This is a hack.
                                .forward(0.001)
                                .waitSeconds(0.25)
                                .addDisplacementMarker(() -> {
//                                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[0]);
//                                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                                    robot.linearSlide.setPower(1);
                                })
                                .back(5)
                                // backing up from cone stack
                                .splineToLinearHeading(new Pose2d(30, 11, Math.toRadians(0)), Math.toRadians(180))

//                                .splineToLinearHeading(new Pose2d(27.5, 5.5, Math.toRadians(0)), Math.toRadians(180))
                                .addDisplacementMarker(() -> {
//                                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[2]);
//                                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                                    robot.linearSlide.setPower(1);
                                })

                                .splineToLinearHeading(new Pose2d(30.001, 11, Math.toRadians(130)), Math.toRadians(130))
                                .splineToLinearHeading(new Pose2d(26.5, 15.5, Math.toRadians(130)), Math.toRadians(130))

//                                .splineToLinearHeading(new Pose2d(28.5, 4.5, Math.toRadians(240)), Math.toRadians(240))

//                                .splineToLinearHeading(new Pose2d(26, 2, Math.toRadians(240)), Math.toRadians(240))
                                .addDisplacementMarker(() -> {
//                                    robot.rightClaw.setPosition(Constants.rightClawOpen);
//                                    robot.leftClaw.setPosition(Constants.leftClawOpen);
                                })
                                .waitSeconds(0.25)
                                .back(5)

                                .addDisplacementMarker(() -> {
//                                    robot.linearSlide.setTargetPosition(0);
//                                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                                    robot.linearSlide.setPower(1);
                                })
                                .splineToLinearHeading(new Pose2d(35, 8, Math.toRadians(90)), Math.toRadians(90))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}