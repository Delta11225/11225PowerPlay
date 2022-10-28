package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware22;
import org.firstinspires.ftc.teamcode.util.Hardware23;

import kotlin.NotImplementedError;

// TODO add a utility method that automatically generates trajectories that go from a tile to another tile
// TODO while avoiding junctions
/*
 * Ex: Navigate from cell A1 to cell B3
 */
public class TrajectoryGenerator {
    private final Hardware23 robot;
    private final SampleMecanumDrive drive;
    private AutoState autoState;
    private ParkingPosition parkPos;

    public TrajectoryGenerator(Hardware23 robot, AutoState autoState, ParkingPosition parkPos) {
        this.robot = robot;
        this.drive = robot.drive;
        this.autoState = autoState;
        this.parkPos = parkPos;
    }
    // FIXME most of this (everything aside from parking) can to be called during init, but parking methods
    // must be called after init as we need to know the color of the signal sleeve. Unfortunately, we'll just have
    // to deal with that and break it up, as otherwise generating trajectories takes forever
    public TrajectorySequence generateTrajectories() {
        TrajectorySequenceBuilder gen = null;
        switch (autoState.color) {
            case RED:
                return getRedTrajectories();
            case BLUE:
                return getBlueTrajectories();
        }
        return null;
    }

    private TrajectorySequence getBlueTrajectories() {
        TrajectorySequenceBuilder gen = null;
        switch (autoState.position) {
            case FRONT:
                Pose2d startPose = new Pose2d(-40, 70-(12.25/2.0), Math.toRadians(270));
                drive.setPoseEstimate(startPose);
                gen = drive.trajectorySequenceBuilder(startPose);
                gen.addDisplacementMarker(() -> {
                    robot.rightClaw.setPosition(Constants.rightClawClosed);
                    robot.leftClaw.setPosition(Constants.leftClawClosed);
                });
                gen.addDisplacementMarker(() -> {
                   robot.linearSlide.setTargetPosition(Constants.liftEncoderLow);
                   robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                   robot.linearSlide.setPower(1);
                });
                gen.splineToLinearHeading(new Pose2d(-25, 53, Math.toRadians(300)), Math.toRadians(300))
                        .addDisplacementMarker(() -> {
                            robot.leftClaw.setPosition(Constants.leftClawOpen);
                            robot.rightClaw.setPosition(Constants.rightClawOpen);
                });
                gen.splineToLinearHeading(new Pose2d(-35, 60, Math.toRadians(270)), Math.toRadians(270));
                gen.addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(0);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                });
//                break;

                switch (parkPos) {
                    case ONE:
                        gen.strafeLeft(35-16)
                                .splineToConstantHeading(new Vector2d(-12, 36), Math.toRadians(270));
                        break;
                    case TWO:
                        gen.forward(70-(12.25/2.0) - 36);
                        break;
                    default:
                        gen.strafeRight(35-16)
//                                .splineToSplineHeading(new Pose2d(-53, 32, Math.toRadians(300)), Math.toRadians(300))
                                .splineToConstantHeading(new Vector2d(-59, 36), Math.toRadians(270));
                        break;
                }
                break;
            case BACK:
                startPose = new Pose2d(35, 70-(12.25/2.0), Math.toRadians(270));
                drive.setPoseEstimate(startPose);
                gen = drive.trajectorySequenceBuilder(startPose);
                switch (parkPos) {
                    case ONE:
                        gen.strafeLeft(35-16)
                                .splineToConstantHeading(new Vector2d(59, 36), Math.toRadians(270));
                        break;
                    case TWO:
                        gen.forward(70-(12.25/2.0) - 36);
                        break;
                    default:
                        gen.strafeRight(35-16)
                                .splineToConstantHeading(new Vector2d(12, 36), Math.toRadians(270));
                        break;
                }
                break;
        }
        return gen.build();
    }

    private TrajectorySequence getRedTrajectories() {
        TrajectorySequenceBuilder gen = null;
        switch (autoState.position) {
            case FRONT:
                Pose2d startPose = new Pose2d(-35, -(70-(12.25/2.0)), Math.toRadians(90));
                drive.setPoseEstimate(startPose);
                gen = drive.trajectorySequenceBuilder(startPose);
                switch (parkPos) {
                    case ONE:
                        gen.strafeLeft(35-16)
                                .splineToConstantHeading(new Vector2d(-59, -36), Math.toRadians(90));
                        break;
                    case TWO:
                        gen.forward(70-(12.25/2.0) - 36);
                        break;
                    default:
                        gen.strafeRight(35-16)
                                .splineToConstantHeading(new Vector2d(-12, -36), Math.toRadians(90));
                        break;
                }
                break;
            case BACK:
                startPose = new Pose2d(35, -(70-(12.25/2.0)), Math.toRadians(90));
                drive.setPoseEstimate(startPose);
                gen = drive.trajectorySequenceBuilder(startPose);
                switch (parkPos) {
                    case ONE:
                        gen.strafeLeft(35-16)
                                .splineToConstantHeading(new Vector2d(12, -36), Math.toRadians(90));
                        break;
                    case TWO:
                        gen.forward(70-(12.25/2.0) - 36);
                        break;
                    default:
                        gen.strafeRight(35-16)
                                .splineToConstantHeading(new Vector2d(59, -36), Math.toRadians(90));
                        break;
                }
                break;
        }
        return gen.build();
    }
}
