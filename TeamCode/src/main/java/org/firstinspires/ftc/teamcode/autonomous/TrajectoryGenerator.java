package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
    private final Telemetry telemetry;

    private HashMap<TrajectoryState, TrajectorySequence> trajectories;

    public TrajectoryGenerator(Hardware23 robot, Telemetry telemetry) {
        this.robot = robot;
        this.drive = robot.drive;
        this.telemetry = telemetry;

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
        // TODO repeated trajectories will need to be cached to save time
        HashMap<TrajectoryState, TrajectorySequence> trajMap = new HashMap<>();
        for (Color color : new Color[]{Color.BLUE, Color.RED}) {
            for (StartPosition startPos : new StartPosition[]{StartPosition.FRONT, StartPosition.BACK}) {
                Log.d("TrajectoryGenerator", String.format("Generating front traj - %s %s", color, startPos));
                telemetry.addLine(String.format("Generating front traj - %s %s", color, startPos));
                telemetry.update();
                TrajectorySequenceBuilder posColor = prepareStartTrajectories(color, startPos);
                for (ParkingPosition parkPos : new ParkingPosition[]{ParkingPosition.ONE, ParkingPosition.TWO, ParkingPosition.THREE}) {
                    Log.d("TrajectoryGenerator", String.format("Generating parking traj - %s %s %s", color, startPos, parkPos));
                    telemetry.addLine(String.format("Generating parking traj - %s %s %s", color, startPos, parkPos));
                    telemetry.update();
                    TrajectoryState trajState = new TrajectoryState(color, startPos, parkPos);
                    TrajectorySequenceBuilder preparedTraj = prepareParkingTrajectories(posColor, trajState);
                    TrajectorySequence traj = preparedTraj.build();
//                    TrajectorySequence traj = genTrajectory(trajState);
                    trajMap.put(trajState, traj);
//                    Log.d("TrajectoryGenerator", "Traj gen finished");
                }
            }
        }
        return trajMap;
    }

    private TrajectorySequenceBuilder prepareStartTrajectories(Color color, StartPosition startPos) {
        switch (color) {
            case RED:
                return prepareRedStartTrajectories(startPos);
            case BLUE:
                return prepareBlueStartTrajectories(startPos);
        }
        return null;
    }

    private TrajectorySequenceBuilder prepareBlueStartTrajectories(StartPosition startPos) {
        switch (startPos) {
            case FRONT:
                Pose2d startPose = new Pose2d(-40, 70 - (12.25 / 2.0), Math.toRadians(270));
                TrajectorySequenceBuilder gen = drive.trajectorySequenceBuilder(startPose);
                gen.setTurnConstraint(60, 5);
                gen.addDisplacementMarker(() -> {
                            robot.rightClaw.setPosition(Constants.rightClawClosed);
                            robot.leftClaw.setPosition(Constants.leftClawClosed);
                        })
                        .addDisplacementMarker(() -> {
                            robot.linearSlide.setTargetPosition(Constants.liftEncoderLow);
                            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.linearSlide.setPower(1);
                        });
                gen.splineToLinearHeading(new Pose2d(-25, 53, Math.toRadians(300)), Math.toRadians(300))
                        .addDisplacementMarker(() -> {
                            robot.leftClaw.setPosition(Constants.leftClawOpen);
                            robot.rightClaw.setPosition(Constants.rightClawOpen);
                        })
                        .splineToLinearHeading(new Pose2d(-35, 60, Math.toRadians(270)), Math.toRadians(270))
                        .addDisplacementMarker(() -> {
                            robot.linearSlide.setTargetPosition(0);
                            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.linearSlide.setPower(1);
                        })
                        .strafeTo(new Vector2d(-14.2, 58.5))
                        .splineToConstantHeading(new Vector2d(-12.2, 58.5), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(-12.2, 9), Math.toRadians(270))
                        .turn(Math.toRadians(-90))
//                    .splineTo(new Vector2d(-14.2, 10), Math.toRadians(180))
                        .addDisplacementMarker(() -> {
                            robot.linearSlide.setTargetPosition(Constants.liftEncoderConeStack[0]);
                            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.linearSlide.setPower(1);
                        })
                        .splineToConstantHeading(new Vector2d(-60, 9), Math.toRadians(180))
                        .waitSeconds(0.25)
                        .addDisplacementMarker(() -> {
                            robot.rightClaw.setPosition(Constants.rightClawClosed);
                            robot.leftClaw.setPosition(Constants.leftClawClosed);
                        })
                        // Do not ever do this. This is a hack.
                        .forward(0.001)
                        .waitSeconds(0.25)
                        .addDisplacementMarker(() -> {
                            robot.linearSlide.setTargetPosition(Constants.liftEncoderLow);
                            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.linearSlide.setPower(1);
                        })
                        .back(12)
                        // FIXME The 17 here should probably be a 16
                        .splineToLinearHeading(new Pose2d(-50.5, 16, Math.toRadians(45)), Math.toRadians(45))
                        .addDisplacementMarker(() -> {
                            robot.rightClaw.setPosition(Constants.rightClawOpen);
                            robot.leftClaw.setPosition(Constants.leftClawOpen);
                        })
                        .splineToLinearHeading(new Pose2d(-56, 12.5, Math.toRadians(90)), Math.toRadians(90))
                        .addDisplacementMarker(() -> {
                            robot.linearSlide.setTargetPosition(0);
                            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.linearSlide.setPower(1);
                        });

                return gen;
            case BACK:
                startPose = new Pose2d(35, 70 - (12.25 / 2.0), Math.toRadians(270));
                gen = drive.trajectorySequenceBuilder(startPose);
                return gen;
        }
        return null;
    }

    private TrajectorySequenceBuilder prepareRedStartTrajectories(StartPosition startPos) {
        switch (startPos) {
            case FRONT:
                Pose2d startPose = new Pose2d(-35, -(70 - (12.25 / 2.0)), Math.toRadians(90));
                TrajectorySequenceBuilder gen = drive.trajectorySequenceBuilder(startPose);

                return gen;
            case BACK:
                startPose = new Pose2d(35, -(70 - (12.25 / 2.0)), Math.toRadians(90));
                gen = drive.trajectorySequenceBuilder(startPose);

                return gen;
        }
        return null;
    }

    private TrajectorySequenceBuilder prepareParkingTrajectories(TrajectorySequenceBuilder posColor, TrajectoryState trajState) {
        switch (trajState.color) {
            case RED:
                return prepareRedParkingTrajectories(posColor, trajState);
            case BLUE:
                return prepareBlueParkingTrajectories(posColor, trajState);
        }
        return null;
    }

    private TrajectorySequenceBuilder prepareBlueParkingTrajectories(TrajectorySequenceBuilder gen, TrajectoryState trajState) {
        switch (trajState.startPosition) {
            case FRONT:
                switch (trajState.parkPos) {
                    case ONE:
                        gen.strafeTo(new Vector2d(-14, 12.5))
                                .splineToConstantHeading(new Vector2d(-12.2, 14), Math.toRadians(90))
                                .splineTo(new Vector2d(-12.2, 34), Math.toRadians(90));
//                        gen.strafeLeft(35-16)
//                                .splineToConstantHeading(new Vector2d(-12, 36), Math.toRadians(270));
                        break;
                    case TWO:
                        gen.strafeTo(new Vector2d(-36, 12.5))
                                .splineToConstantHeading(new Vector2d(-34.5, 14), Math.toRadians(90))
                                .splineTo(new Vector2d(-34.5, 34), Math.toRadians(90));
//                        gen.forward(70-(12.25/2.0) - 36);
                        break;
                    default:
                        gen.forward(20);
//                        gen.strafeRight(35-16)
//                                .splineToSplineHeading(new Pose2d(-53, 32, Math.toRadians(300)), Math.toRadians(300))
//                                .splineToConstantHeading(new Vector2d(-59, 36), Math.toRadians(270));
                        break;
                }
                return gen;
            case BACK:
                switch (trajState.parkPos) {
                    case ONE:
                        gen.strafeLeft(35 - 16)
                                .splineToConstantHeading(new Vector2d(59, 36), Math.toRadians(270));
                        break;
                    case TWO:
                        gen.forward(70 - (12.25 / 2.0) - 36);
                        break;
                    default:
                        gen.strafeRight(35 - 16)
                                .splineToConstantHeading(new Vector2d(12, 36), Math.toRadians(270));
                        break;
                }
                return gen;
        }
        return null;
    }

    private TrajectorySequenceBuilder prepareRedParkingTrajectories(TrajectorySequenceBuilder gen, TrajectoryState trajState) {
        switch (trajState.startPosition) {
            case FRONT:
                switch (trajState.parkPos) {
                    case ONE:
                        gen.strafeLeft(35 - 16)
                                .splineToConstantHeading(new Vector2d(-59, -36), Math.toRadians(90));
                        break;
                    case TWO:
                        gen.forward(70 - (12.25 / 2.0) - 36);
                        break;
                    default:
                        gen.strafeRight(35 - 16)
                                .splineToConstantHeading(new Vector2d(-12, -36), Math.toRadians(90));
                        break;
                }
                return gen;
            case BACK:
                switch (trajState.parkPos) {
                    case ONE:
                        gen.strafeLeft(35 - 16)
                                .splineToConstantHeading(new Vector2d(12, -36), Math.toRadians(90));
                        break;
                    case TWO:
                        gen.forward(70 - (12.25 / 2.0) - 36);
                        break;
                    default:
                        gen.strafeRight(35 - 16)
                                .splineToConstantHeading(new Vector2d(59, -36), Math.toRadians(90));
                        break;
                }
                return gen;
        }
        return null;
    }

    @Deprecated
    public TrajectorySequence genTrajectory(TrajectoryState state) {
        switch (state.color) {
            case RED:
                return genRedTrajectories(state);
            case BLUE:
                return genBlueTrajectories(state);
        }
        return null;
    }

    @Deprecated
    private TrajectorySequence genBlueTrajectories(TrajectoryState state) {
        TrajectorySequenceBuilder gen = null;
        StartPosition startPos = state.startPosition;
        ParkingPosition parkPos = state.parkPos;
        switch (startPos) {
            case FRONT:
                Pose2d startPose = new Pose2d(-40, 70 - (12.25 / 2.0), Math.toRadians(270));
                gen = drive.trajectorySequenceBuilder(startPose);
                gen.setTurnConstraint(60, 5);
                gen.addDisplacementMarker(() -> {
                            robot.rightClaw.setPosition(Constants.rightClawClosed);
                            robot.leftClaw.setPosition(Constants.leftClawClosed);
                        })
                        .addDisplacementMarker(() -> {
                            robot.linearSlide.setTargetPosition(Constants.liftEncoderLow);
                            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.linearSlide.setPower(1);
                        });
                gen.splineToLinearHeading(new Pose2d(-25, 53, Math.toRadians(300)), Math.toRadians(300))
                        .addDisplacementMarker(() -> {
                            robot.leftClaw.setPosition(Constants.leftClawOpen);
                            robot.rightClaw.setPosition(Constants.rightClawOpen);
                        })
                        .splineToLinearHeading(new Pose2d(-35, 60, Math.toRadians(270)), Math.toRadians(270))
                        .addDisplacementMarker(() -> {
                            robot.linearSlide.setTargetPosition(0);
                            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.linearSlide.setPower(1);
                        })
                        .strafeTo(new Vector2d(-14.2, 58.5))
                        .splineToConstantHeading(new Vector2d(-12.2, 58.5), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(-12.2, 9), Math.toRadians(270))
                        .turn(Math.toRadians(-90))
//                    .splineTo(new Vector2d(-14.2, 10), Math.toRadians(180))
                        .addDisplacementMarker(() -> {
                            robot.linearSlide.setTargetPosition(Constants.liftEncoderConeStack[0]);
                            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.linearSlide.setPower(1);
                        })
                        .splineToConstantHeading(new Vector2d(-60, 9), Math.toRadians(180))
                        .waitSeconds(0.25)
                        .addDisplacementMarker(() -> {
                            robot.rightClaw.setPosition(Constants.rightClawClosed);
                            robot.leftClaw.setPosition(Constants.leftClawClosed);
                        })
                        // Do not ever do this. This is a hack.
                        .forward(0.001)
                        .waitSeconds(0.25)
                        .addDisplacementMarker(() -> {
                            robot.linearSlide.setTargetPosition(Constants.liftEncoderLow);
                            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.linearSlide.setPower(1);
                        })
                        .back(12)
                        // FIXME The 17 here should probably be a 16
                        .splineToLinearHeading(new Pose2d(-50.5, 16, Math.toRadians(45)), Math.toRadians(45))
                        .addDisplacementMarker(() -> {
                            robot.rightClaw.setPosition(Constants.rightClawOpen);
                            robot.leftClaw.setPosition(Constants.leftClawOpen);
                        })
                        .splineToLinearHeading(new Pose2d(-56, 12.5, Math.toRadians(90)), Math.toRadians(90))
                        .addDisplacementMarker(() -> {
                            robot.linearSlide.setTargetPosition(0);
                            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.linearSlide.setPower(1);
                        });

//                gen.splineToLinearHeading(new Pose2d(-25, 53, Math.toRadians(300)), Math.toRadians(300))
//                gen.splineToLinearHeading(new Pose2d(-35, 60, Math.toRadians(270)), Math.toRadians(270));                gen
//                    .splineToLinearHeading(new Pose2d(-28, 53.8, Math.toRadians(300)), Math.toRadians(300))
//                    .splineToLinearHeading(new Pose2d(-27.7, 58.5, Math.toRadians(270)), Math.toRadians(270))
//                    .strafeTo(new Vector2d(-14.2, 58.5))
//                    .splineToConstantHeading(new Vector2d(-12.2, 58.5), Math.toRadians(270))
//                    .splineToConstantHeading(new Vector2d(-12.2, 14), Math.toRadians(270))
//                    .splineTo(new Vector2d(-14.2, 12), Math.toRadians(180))
//                    .splineToConstantHeading(new Vector2d(-59, 12), Math.toRadians(180))
//                    .addDisplacementMarker(() -> {
//                        robot.rightClaw.setPosition(Constants.rightClawClosed);
//                        robot.leftClaw.setPosition(Constants.leftClawClosed);
//                        robot.linearSlide.setTargetPosition(Constants.liftEncoderLow);
//                        robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        robot.linearSlide.setPower(1);
//                    })
//                    .back(10)
//                    .splineToLinearHeading(new Pose2d(-52, 17.6, Math.toRadians(45)), Math.toRadians(45))
//                    .addDisplacementMarker(() -> {
//                        robot.rightClaw.setPosition(Constants.rightClawOpen);
//                        robot.leftClaw.setPosition(Constants.rightClawOpen);
//                    })
//                    .splineToLinearHeading(new Pose2d(-54.8, 12.5, Math.toRadians(90)), Math.toRadians(90))
//                    .addDisplacementMarker(() -> {
//                        robot.linearSlide.setTargetPosition(0);
//                        robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        robot.linearSlide.setPower(1);
//                    })
//                    .strafeTo(new Vector2d(-34.5, 12.5));
//                break;

                switch (parkPos) {
                    case ONE:
                        gen.strafeTo(new Vector2d(-14, 12.5))
                                .splineToConstantHeading(new Vector2d(-12.2, 14), Math.toRadians(90))
                                .splineTo(new Vector2d(-12.2, 34), Math.toRadians(90));
//                        gen.strafeLeft(35-16)
//                                .splineToConstantHeading(new Vector2d(-12, 36), Math.toRadians(270));
                        break;
                    case TWO:
                        gen.strafeTo(new Vector2d(-36, 12.5))
                                .splineToConstantHeading(new Vector2d(-34.5, 14), Math.toRadians(90))
                                .splineTo(new Vector2d(-34.5, 34), Math.toRadians(90));
//                        gen.forward(70-(12.25/2.0) - 36);
                        break;
                    default:
                        gen.forward(20);
//                        gen.strafeRight(35-16)
//                                .splineToSplineHeading(new Pose2d(-53, 32, Math.toRadians(300)), Math.toRadians(300))
//                                .splineToConstantHeading(new Vector2d(-59, 36), Math.toRadians(270));
                        break;
                }
                break;
            case BACK:
                startPose = new Pose2d(35, 70 - (12.25 / 2.0), Math.toRadians(270));
                gen = drive.trajectorySequenceBuilder(startPose);
                switch (parkPos) {
                    case ONE:
                        gen.strafeLeft(35 - 16)
                                .splineToConstantHeading(new Vector2d(59, 36), Math.toRadians(270));
                        break;
                    case TWO:
                        gen.forward(70 - (12.25 / 2.0) - 36);
                        break;
                    default:
                        gen.strafeRight(35 - 16)
                                .splineToConstantHeading(new Vector2d(12, 36), Math.toRadians(270));
                        break;
                }
                break;
        }
        return gen.build();
    }

    @Deprecated
    private TrajectorySequence genRedTrajectories(TrajectoryState state) {
        TrajectorySequenceBuilder gen = null;
        StartPosition startPos = state.startPosition;
        ParkingPosition parkPos = state.parkPos;
        switch (startPos) {
            case FRONT:
                Pose2d startPose = new Pose2d(-35, -(70 - (12.25 / 2.0)), Math.toRadians(90));
                gen = drive.trajectorySequenceBuilder(startPose);
                switch (parkPos) {
                    case ONE:
                        gen.strafeLeft(35 - 16)
                                .splineToConstantHeading(new Vector2d(-59, -36), Math.toRadians(90));
                        break;
                    case TWO:
                        gen.forward(70 - (12.25 / 2.0) - 36);
                        break;
                    default:
                        gen.strafeRight(35 - 16)
                                .splineToConstantHeading(new Vector2d(-12, -36), Math.toRadians(90));
                        break;
                }
                break;
            case BACK:
                startPose = new Pose2d(35, -(70 - (12.25 / 2.0)), Math.toRadians(90));
                gen = drive.trajectorySequenceBuilder(startPose);
                switch (parkPos) {
                    case ONE:
                        gen.strafeLeft(35 - 16)
                                .splineToConstantHeading(new Vector2d(12, -36), Math.toRadians(90));
                        break;
                    case TWO:
                        gen.forward(70 - (12.25 / 2.0) - 36);
                        break;
                    default:
                        gen.strafeRight(35 - 16)
                                .splineToConstantHeading(new Vector2d(59, -36), Math.toRadians(90));
                        break;
                }
                break;
        }
        return gen.build();
    }

    private static class TrajectoryState {
        public final Color color;
        public final StartPosition startPosition;
        public final ParkingPosition parkPos;

        TrajectoryState() {
            this.color = null;
            this.startPosition = null;
            this.parkPos = null;
        }

        TrajectoryState(Color color, StartPosition startPosition, ParkingPosition parkPos) {
            this.color = color;
            this.startPosition = startPosition;
            this.parkPos = parkPos;
        }

        public boolean hasSameValue(TrajectoryState state) {
            return this.color == state.color && this.startPosition == state.startPosition && this.parkPos == state.parkPos;
        }
    }
}
