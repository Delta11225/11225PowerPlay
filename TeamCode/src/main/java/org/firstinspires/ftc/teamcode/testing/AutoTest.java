package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware23;

@Autonomous
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware23 robot = new Hardware23(hardwareMap);
        Pose2d startPose = new Pose2d(-40, 70 - (12.25 / 2.0), Math.toRadians(270));
        robot.drive.setPoseEstimate(startPose);
        TrajectorySequence test = robot.drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    robot.rightClaw.setPosition(Constants.rightClawClosed);
                    robot.leftClaw.setPosition(Constants.leftClawClosed);
                })
                .strafeTo(new Vector2d(-12.5, 65))
                .splineToConstantHeading(new Vector2d(-10, 57.1), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[2]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .splineToLinearHeading(new Pose2d(-5.6, 31, Math.toRadians(300)), Math.toRadians(300))
                .addDisplacementMarker(() -> {
                    robot.leftClaw.setPosition(Constants.leftClawOpen);
                    robot.rightClaw.setPosition(Constants.rightClawOpen);
                    robot.linearSlide.setTargetPosition(0);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .waitSeconds(1)
/*
             // Back up
              .splineToLinearHeading(new Pose2d(-13, 30, Math.toRadians(270)), Math.toRadians(270))

             // Go to turn
              .splineToLinearHeading(new Pose2d(-16, 12, Math.toRadians(180)), Math.toRadians(180))
              .addDisplacementMarker(() -> {
                 robot.linearSlide.setTargetPosition(Constants.getLiftEncoderConeStack()[0]);
                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.linearSlide.setPower(1);
              })
              .splineToConstantHeading(new Vector2d(-60, 9), Math.toRadians(180))
              //.waitSeconds(0.25)
              .addDisplacementMarker(() -> {
                 robot.rightClaw.setPosition(Constants.rightClawClosed);
                 robot.leftClaw.setPosition(Constants.leftClawClosed);
             })
              // Do not ever do this. This is a hack.
              .forward(0.001)
              //.waitSeconds(0.25)
              .addDisplacementMarker(() -> {
                 robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[0]);
                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.linearSlide.setPower(1);
              })
              .back(20)
             .addDisplacementMarker(() -> {
                 robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[2]);
                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.linearSlide.setPower(1);
              })
              .splineToLinearHeading(new Pose2d(-28.5, 8.5, Math.toRadians(300)), Math.toRadians(300))
              .addDisplacementMarker(() -> {
                robot.rightClaw.setPosition(Constants.rightClawOpen);
                 robot.leftClaw.setPosition(Constants.leftClawOpen);
             })
              .back(10)
              .addDisplacementMarker(() -> {
                 robot.linearSlide.setTargetPosition(0);
                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                 robot.linearSlide.setPower(1);
              })
             .forward(0.001)*/
                .build();


        waitForStart();

        if (isStopRequested()) return;

        robot.drive.followTrajectorySequence(test);
    }
}

