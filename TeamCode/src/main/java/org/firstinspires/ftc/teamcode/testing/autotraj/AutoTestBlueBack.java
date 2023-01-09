package org.firstinspires.ftc.teamcode.testing.autotraj;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware23;

@Autonomous
public class AutoTestBlueBack extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware23 robot = new Hardware23(hardwareMap);
        Pose2d startPose = new Pose2d(29.5, 70 - (12.25 / 2.0), Math.toRadians(270));
        robot.drive.setPoseEstimate(startPose);
        TrajectorySequence test = robot.drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    robot.rightClaw.setPosition(Constants.rightClawClosed);
                    robot.leftClaw.setPosition(Constants.leftClawClosed);
                })
                .strafeTo(new Vector2d(12.5, 65))
                .splineToConstantHeading(new Vector2d(10, 57.1), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[2]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .splineToLinearHeading(new Pose2d(2, 32, Math.toRadians(240)), Math.toRadians(240))

                .splineToLinearHeading(new Pose2d(1.5, 29.5, Math.toRadians(240)), Math.toRadians(240))
                .addDisplacementMarker(() -> {
                    robot.leftClaw.setPosition(Constants.leftClawOpen);
                    robot.rightClaw.setPosition(Constants.rightClawOpen);
                })
                .waitSeconds(0.25)

                // Back up
                .splineToLinearHeading(new Pose2d(6, 36, Math.toRadians(240)), Math.toRadians(240))
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(0);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })

             // square up
              .splineToLinearHeading(new Pose2d(10, 30, Math.toRadians(270)), Math.toRadians(270))

             // Go to turn
              .splineToLinearHeading(new Pose2d(12, 8, Math.toRadians(0)), Math.toRadians(0))
              .addDisplacementMarker(() -> {
                 robot.linearSlide.setTargetPosition(Constants.getLiftEncoderConeStack()[0]);
                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.linearSlide.setPower(1);
              })
                //approach cone stack
              .splineToConstantHeading(new Vector2d(57, 8), Math.toRadians(0))
              .waitSeconds(0.25)

              .addDisplacementMarker(() -> {
                 robot.rightClaw.setPosition(Constants.rightClawClosed);
                 robot.leftClaw.setPosition(Constants.leftClawClosed);
             })
              // Do not ever do this. This is a hack.
              .forward(0.001)
              .waitSeconds(0.25)
              .addDisplacementMarker(() -> {
                 robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[0]);
                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.linearSlide.setPower(1);
              })
              .back(5)

             .splineToLinearHeading(new Pose2d(30, 8, Math.toRadians(0)), Math.toRadians(0))
             .addDisplacementMarker(() -> {
                 robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[2]);
                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.linearSlide.setPower(1);
              })

             .splineToLinearHeading(new Pose2d(30.5, 5.5, Math.toRadians(240)), Math.toRadians(240))

              .splineToLinearHeading(new Pose2d(28, 3, Math.toRadians(240)), Math.toRadians(240))
              .addDisplacementMarker(() -> {
                robot.rightClaw.setPosition(Constants.rightClawOpen);
                 robot.leftClaw.setPosition(Constants.leftClawOpen);
             })
                .waitSeconds(0.25)
              .back(5)

              .addDisplacementMarker(() -> {
                 robot.linearSlide.setTargetPosition(0);
                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.linearSlide.setPower(1);
              })
              .splineToLinearHeading(new Pose2d(35, 8, Math.toRadians(270)), Math.toRadians(270))


              .build();


        waitForStart();

        if (isStopRequested()) return;

        robot.drive.followTrajectorySequence(test);
    }
}

