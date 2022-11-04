package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware23;

import java.util.HashMap;

// TODO add a utility method that automatically generates trajectories that go from a tile to another tile
// TODO while avoiding junctions
/*
 * Ex: Navigate from cell A1 to cell B3
 */
public class TrajectoryGenerator {
    private final Hardware23 robot;
    private final SampleMecanumDrive drive;

    private HashMap<TrajectoryState, TrajectorySequence> trajectories;

    public TrajectoryGenerator(Hardware23 robot) {
        this.robot = robot;
        this.drive = robot.drive;

        this.trajectories = generateAllTrajectories();
    }

    public TrajectorySequence getAppropriateTrajectory(AutoState autoState, ParkingPosition parkPos) {
        TrajectoryState tempTrajState = new TrajectoryState(autoState.color, autoState.position, parkPos);
        TrajectoryState mapTrajState = new TrajectoryState();

        for (TrajectoryState trajState : trajectories.keySet()) {
            if (tempTrajState.hasSameValue(trajState)) {
                mapTrajState = trajState;
                break;
            }
        }

        TrajectorySequence traj = trajectories.getOrDefault(mapTrajState, null);
        if (traj == null) {
            throw new IllegalStateException("WTF? How? You somehow have provide an auto state that there" +
                    "aren't trajectories for.");
        }
        drive.setPoseEstimate(traj.start());
        return traj;
    }

    private HashMap<TrajectoryState, TrajectorySequence> generateAllTrajectories() {
        HashMap<TrajectoryState, TrajectorySequence> trajMap = new HashMap<>();
        for (Color color : new Color[]{Color.BLUE, Color.RED}) {
            for (StartPosition startPos : new StartPosition[]{StartPosition.FRONT, StartPosition.FRONT})
                for (ParkingPosition parkPos : new ParkingPosition[]{ParkingPosition.ONE, ParkingPosition.TWO, ParkingPosition.THREE}) {
                    TrajectoryState trajState = new TrajectoryState(color, startPos, parkPos);
                    TrajectorySequence traj = genTrajectory(trajState);
                    trajMap.put(trajState, traj);
            }
        }
        return trajMap;
    }
    // FIXME most of this (everything aside from parking) can to be called during init, but parking methods
    // must be called after init as we need to know the color of the signal sleeve. Unfortunately, we'll just have
    // to deal with that and break it up, as otherwise generating trajectories takes forever
    // TODO it would be easier to just generate all of the possible trajectories and follow the appropriate one
    public TrajectorySequence genTrajectory(TrajectoryState state) {
        switch (state.color) {
            case RED:
                return genRedTrajectories(state);
            case BLUE:
                return genBlueTrajectories(state);
        }
        return null;
    }

    private TrajectorySequence genBlueTrajectories(TrajectoryState state) {
        TrajectorySequenceBuilder gen = null;
        StartPosition startPos = state.position;
        ParkingPosition parkPos = state.parkPos;
        switch (startPos) {
            case FRONT:
                Pose2d startPose = new Pose2d(-40, 70-(12.25/2.0), Math.toRadians(270));
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
                gen
                    .splineToLinearHeading(new Pose2d(-28, 53.8, Math.toRadians(300)), Math.toRadians(300))
                    .splineToLinearHeading(new Pose2d(-27.7, 58.5, Math.toRadians(270)), Math.toRadians(270))
                    .strafeTo(new Vector2d(-14.2, 58.5))
                    .splineToConstantHeading(new Vector2d(-12.2, 58.5), Math.toRadians(270))
                    .splineToConstantHeading(new Vector2d(-12.2, 14), Math.toRadians(270))
                    .splineTo(new Vector2d(-14.2, 12), Math.toRadians(180))
                    .addDisplacementMarker(() -> {
                        robot.linearSlide.setTargetPosition(Constants.liftEncoderConeStack[0]);
                        robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.linearSlide.setPower(1);
                    })
                    .splineToConstantHeading(new Vector2d(-59, 12), Math.toRadians(180))
                    .addDisplacementMarker(() -> {
                        robot.rightClaw.setPosition(Constants.rightClawClosed);
                        robot.leftClaw.setPosition(Constants.leftClawClosed);
                        robot.linearSlide.setTargetPosition(Constants.liftEncoderLow);
                        robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.linearSlide.setPower(1);
                    })
                    .back(10)
                    .splineToLinearHeading(new Pose2d(-52, 17.6, Math.toRadians(45)), Math.toRadians(45))
                    .addDisplacementMarker(() -> {
                        robot.rightClaw.setPosition(Constants.rightClawOpen);
                        robot.leftClaw.setPosition(Constants.rightClawOpen);
                    })
                    .splineToLinearHeading(new Pose2d(-54.8, 12.5, Math.toRadians(90)), Math.toRadians(90))
                    .addDisplacementMarker(() -> {
                        robot.linearSlide.setTargetPosition(0);
                        robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.linearSlide.setPower(1);
                    })
                    .strafeTo(new Vector2d(-34.5, 12.5));
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

    private TrajectorySequence genRedTrajectories(TrajectoryState state) {
        TrajectorySequenceBuilder gen = null;
        StartPosition startPos = state.position;
        ParkingPosition parkPos = state.parkPos;
        switch (startPos) {
            case FRONT:
                Pose2d startPose = new Pose2d(-35, -(70-(12.25/2.0)), Math.toRadians(90));
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

    private static class TrajectoryState {
        public final Color color;
        public final StartPosition position;
        public final ParkingPosition parkPos;

        TrajectoryState() {
           this.color = null;
           this.position = null;
           this.parkPos = null;
        }

        TrajectoryState(Color color, StartPosition position, ParkingPosition parkPos) {
            this.color = color;
            this.position = position;
            this.parkPos = parkPos;
        }

        public boolean hasSameValue(TrajectoryState state) {
            return this.color == state.color && this.position == state.position && this.parkPos == state.parkPos;
        }
    }
}
