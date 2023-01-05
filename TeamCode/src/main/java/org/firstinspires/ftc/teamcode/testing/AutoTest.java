package org.firstinspires.ftc.teamcode.testing;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware23;

@Autonomous
public class AutoTest extends LinearOpMode {
   @Override
   public void runOpMode() throws InterruptedException {

      Hardware23 robot = new Hardware23(hardwareMap);
      SampleMecanumDrive drive = robot.drive;

      waitForStart();

      Pose2d startPose = new Pose2d(-40, 70 - (12.25 / 2.0), Math.toRadians(270));
      drive.setPoseEstimate(startPose);
      TrajectorySequence gen = drive.trajectorySequenceBuilder(startPose)
//              .setTurnConstraint(60, 5)
//              .addDisplacementMarker(() -> {
//                 robot.rightClaw.setPosition(Constants.rightClawClosed);
//                 robot.leftClaw.setPosition(Constants.leftClawClosed);
//              })
              .strafeTo(new Vector2d(-12.5, 65))
//              .splineToConstantHeading(new Vector2d(-10, 57.1), Math.toRadians(270))
////              .addDisplacementMarker(() -> {
////                 robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[2]);
////                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                 robot.linearSlide.setPower(1);
////              })
//              .splineToLinearHeading(new Pose2d(-5.6, 31, Math.toRadians(300)), Math.toRadians(300))
////              .addDisplacementMarker(() -> {
////                 robot.leftClaw.setPosition(Constants.leftClawOpen);
////                 robot.rightClaw.setPosition(Constants.rightClawOpen);
////                 robot.linearSlide.setTargetPosition(0);
////                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                 robot.linearSlide.setPower(1);
////              })
//
//              // Back up
//              .splineToLinearHeading(new Pose2d(-13, 30, Math.toRadians(270)), Math.toRadians(270))
//
//              // Go to turn
//              .splineToLinearHeading(new Pose2d(-16, 12, Math.toRadians(180)), Math.toRadians(180))
////              .addDisplacementMarker(() -> {
////                 robot.linearSlide.setTargetPosition(Constants.getLiftEncoderConeStack()[0]);
////                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                 robot.linearSlide.setPower(1);
////              })
//              .splineToConstantHeading(new Vector2d(-60, 9), Math.toRadians(180))
//              .waitSeconds(0.25)
////              .addDisplacementMarker(() -> {
////                 robot.rightClaw.setPosition(Constants.rightClawClosed);
////                 robot.leftClaw.setPosition(Constants.leftClawClosed);
////              })
//              // Do not ever do this. This is a hack.
//              .forward(0.001)
              //.waitSeconds(0.25)
////              .addDisplacementMarker(() -> {
////                 robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[0]);
////                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                 robot.linearSlide.setPower(1);
////              })
//              .back(20)
////              .addDisplacementMarker(() -> {
////                 robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[2]);
////                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                 robot.linearSlide.setPower(1);
////              })
//              .splineToLinearHeading(new Pose2d(-28.5, 8.5, Math.toRadians(300)), Math.toRadians(300))
////              .addDisplacementMarker(() -> {
////                 robot.rightClaw.setPosition(Constants.rightClawOpen);
////                 robot.leftClaw.setPosition(Constants.leftClawOpen);
////              })
//              .back(10)
////              .addDisplacementMarker(() -> {
////                 robot.linearSlide.setTargetPosition(0);
////                 robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                 robot.linearSlide.setPower(1);
////              })
//              .forward(0.001)
              .build();
      drive.followTrajectorySequence(gen);

//      while (true) {
//         Log.v("trajectory", String.format("Action: t %4.2f / %4.2f: txWorldTarget.x %4.2f, txWorldTarget.y %4.2f, txWorldTarget.rotation (%4.2f, %4.2f)",
//                 t, profile.duration, txWorldTarget.trans.x.value(), txWorldTarget.trans.y.value(), txWorldTarget.rot.real.value(),  txWorldTarget.rot.imag.value()));
//
//         Log.v("trajectory", String.format("Action: t %4.2f / %4.2f: txActualWorld.x %4.2f, txActualWorld.y %4.2f, txActualWorld.rotation (%4.2f, %4.2f)",
//                 t, profile.duration, txActualWorld.trans.x.value(), txActualWorld.trans.y.value(), txActualWorld.rot.real.value(),  txActualWorld.rot.imag.value()));
//
//         Log.v("trajectory", String.format("Action: t %4.2f / %4.2f: command.x %4.2f, command.y %4.2f, command.rotation %4.2f",
//                 t, profile.duration, command.transVel.x.value(), command.transVel.y.value(), command.rotVel.value()));
//
//         Log.v("trajectory", String.format("Action: t %4.2f / %4.2f: wheelVels: leftFront %4.2f, leftBack %4.2f, rightBack %4.2f, rightFront %4.2f",
//                 t, profile.duration, wheelVels.leftFront.value(), wheelVels.leftBack.value(), wheelVels.rightBack.value(), wheelVels.rightFront.value()));
//      }
   }
}
