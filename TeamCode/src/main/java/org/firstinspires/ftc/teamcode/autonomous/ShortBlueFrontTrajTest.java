package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware23;

@Autonomous
//@Disabled
public class ShortBlueFrontTrajTest extends LinearOpMode {
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
                .forward(.01)
                .waitSeconds(1.0)
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[0]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                //.strafeTo(new Vector2d(-20,54.375 ))
                .lineToConstantHeading(new Vector2d(-20,54.375))
                .waitSeconds(0.5)
                .addDisplacementMarker(() -> {
                    robot.leftClaw.setPosition(Constants.leftClawOpen);
                    robot.rightClaw.setPosition(Constants.rightClawOpen);
                })
                .forward(0.01)
                .waitSeconds(0.25)
                .strafeTo(new Vector2d(-35,60 ))
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.linearSlideZeroOffset);

                })

                /* BLUE FRONT PARKING 1
                .strafeTo(new Vector2d(-8, 60))
                .splineToLinearHeading(new Pose2d(-8, 33, Math.toRadians(90)), Math.toRadians(270))
                */

                /* BLUE FRONT PARKING 2
                .strafeTo(new Vector2d(-8, 60))
                .splineToLinearHeading(new Pose2d(-8, 33, Math.toRadians(90)), Math.toRadians(270))
                .strafeTo(new Vector2d(-40, 33))
                .strafeTo(new Vector2d(-35, 33))
                 */

                /* BLUE FRONT PARKING 3
                .strafeTo(new Vector2d(-62, 60))
                .splineToLinearHeading(new Pose2d(-62, 33, Math.toRadians(90)), Math.toRadians(270))
                 */

                // 10/1 Parking 1
                //.strafeTo(new Vector2d(6, 60))
                //.splineToLinearHeading(new Pose2d(6, 33, Math.toRadians(90)), Math.toRadians(270))


                // 10/1 Parking 2
                //.strafeTo(new Vector2d(-6, 60))
                // .splineToLinearHeading(new Pose2d(-6, 33, Math.toRadians(90)), Math.toRadians(270))
                // .strafeTo(new Vector2d(-40, 33))
                // .strafeTo(new Vector2d(-32, 33))

                // 10/1 Parking 3
                .strafeTo(new Vector2d(-63, 60))
                .splineToLinearHeading(new Pose2d(-63, 33, Math.toRadians(90)), Math.toRadians(270))


                //.splineToConstantHeading(new Vector2d(-10, 57.1), Math.toRadians(270))

                /*
                .splineToLinearHeading(new Pose2d(-2, 32, Math.toRadians(300)), Math.toRadians(300))

                .splineToLinearHeading(new Pose2d(-.5, 29.5, Math.toRadians(300)), Math.toRadians(300))
                .addDisplacementMarker(() -> {
                    robot.leftClaw.setPosition(Constants.leftClawOpen);
                    robot.rightClaw.setPosition(Constants.rightClawOpen);
                })
                .waitSeconds(0.25)
                // Back up
                .splineToLinearHeading(new Pose2d(-6, 36, Math.toRadians(300)), Math.toRadians(300))
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(0);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                // square up
                .splineToLinearHeading(new Pose2d(-10, 30, Math.toRadians(270)), Math.toRadians(270))

                // Go to turn
                .splineToLinearHeading(new Pose2d(-12, 11, Math.toRadians(180)), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderConeStack()[0]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                // Approach cone stack
                .splineToConstantHeading(new Vector2d(-55.5, 9.5), Math.toRadians(180))
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

                .splineToLinearHeading(new Pose2d(-30, 11, Math.toRadians(180)), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[1]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })

                .splineToLinearHeading(new Pose2d(-29.5, 8.5, Math.toRadians(60)), Math.toRadians(60))
                .splineToLinearHeading(new Pose2d(-27, 15.5, Math.toRadians(60)), Math.toRadians(60))
                .forward(1)
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
                .splineToLinearHeading(new Pose2d(-35, 11, Math.toRadians(90)), Math.toRadians(90))


                .waitSeconds(2
                 */

                .build();

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.followTrajectorySequence(test);
    }
}

