package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.google.gson.annotations.Since;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware23;

import java.util.HashMap;

// FIXME SUPER DUPER ULTRA MEGA IMPORTANT: This class is broken somehow, review
public class TrajectoryGenerator {
    // Various useful variables
    private final Hardware23 robot;
    private final SampleMecanumDrive drive;
    private final Telemetry telemetry;

    /**
     * Stores mapping between Auto State (color, start pos, park pos) and traj sequence to run.
     * Stores an array of start trajectory and parking trajectory for efficiency reasons. It is inefficient to
     * regenerate start traj for each parking position, but we can't use the same start traj for each parking
     * trajectory because there is no good way to copy a TrajectorySequenceBuilder. As a result, we generate start
     * and parking trajectories separately, and store them in an array in this dictionary. It's annoying, but it's what
     * we gotta do.
     */
    private HashMap<TrajectoryState, TrajectorySequence[]> trajectories;

    /**
     * General constructor. Generates all trajectories.
     * @param robot The hardware file corresponding to the robot we are using
     * @param telemetry The telemetry object to direct telem calls to
     */
    public TrajectoryGenerator(Hardware23 robot, Telemetry telemetry) {
        this.robot = robot;
        this.drive = robot.drive;
        this.telemetry = telemetry;

        // Self explanatory. Populates trajectories variable
        this.trajectories = generateAllTrajectories();
    }

    /**
     * Return the trajectory sequence corresponding with the auto state given
     * @param autoState The state that auto is in (color, start pos)
     * @param parkPos The required parking position
     * @return The appropriate trajectory sequence for this auto state
     */
    public TrajectorySequence[] getAppropriateTrajectory(AutoState autoState, ParkingPosition parkPos) {
        Log.d("TrajectoryGenerator", String.format("Requested trajectory - %s %s %S",
                autoState.color, autoState.position, parkPos));
        // A TrajectoryState (combines AutoState and ParkingPosition) that corresponds to the parameters
        TrajectoryState tempTrajState = new TrajectoryState(autoState.color, autoState.position, parkPos);
        // Used later for fetching appropriate value from hashmap
        TrajectoryState mapTrajState = new TrajectoryState();

        // Since hashmaps are not by value but by reference, we need to manually loop through the hashmap to find
        // the key with the same VALUE as the trajectory state we have, and save it
        for (TrajectoryState trajState : trajectories.keySet()) {
            if (tempTrajState.hasSameValue(trajState)) {
                mapTrajState = trajState;
                break;
            }
        }

        // Get the right trajectories with the traj state object we found earlier. If we don't find it, provide null instead of
        // throwing an error
        TrajectorySequence[] traj = trajectories.getOrDefault(mapTrajState, null);
        
        // If traj is null, it means it wasn't in our hashmap, and something has gone wrong. Let the user know.
        if (traj == null) {
            // Throw and error and report the error to the log (debug reasons)
            Log.e("TrajectoryGenerator", "WTF? How? You somehow have provide an auto state that there" +
                    "aren't trajectories for.");
            throw new IllegalStateException("WTF? How? You somehow have provide an auto state that there" +
                    "aren't trajectories for.");
        }

        // Make sure to tell the robot where it is
        drive.setPoseEstimate(traj[0].start());
        return traj;
    }

    /**
     * Loops through all possible auto states and parking methods and generates appropriate trajectories.
     * @return A hashmap with the TrajState as the key and the TrajSequence as the value
     */
    private HashMap<TrajectoryState, TrajectorySequence[]> generateAllTrajectories() {
        HashMap<TrajectoryState, TrajectorySequence[]> trajMap = new HashMap<>();

        // Loop through each possible color and start position
        for (StartPosition startPos : new StartPosition[]{StartPosition.FRONT, StartPosition.BACK}) {
            // Log to the console to tell user what we are working on
            Log.d("TrajectoryGenerator", String.format("Generating start traj - %s", startPos));
            telemetry.addLine(String.format("Generating start traj - %s", startPos));
            telemetry.update();

            // Since the first part of each trajectory (the stuff before parking) is the same,
            // we generate it separately and don't build to avoid wasting a lot of time.
            TrajectorySequence posColorTraj = prepareBlueStartTrajectories(startPos).build();

            // Loop through each parking position
            for (ParkingPosition parkPos : new ParkingPosition[]{ParkingPosition.ONE, ParkingPosition.TWO, ParkingPosition.THREE}) {
                // Have to make this generator here or we do all parking trajectories no matter what
                TrajectorySequenceBuilder parkingGen = drive.trajectorySequenceBuilder(posColorTraj.end());
                // Once again, make sure to log
                Log.d("TrajectoryGenerator", String.format("Generating parking traj - %s %s", startPos, parkPos));
                telemetry.addLine(String.format("Generating parking traj - %s %s", startPos, parkPos));
                telemetry.update();

                // Construct this to pass into parking trajectories method
                TrajectoryState trajState = new TrajectoryState(Color.BLUE, startPos, parkPos);

                // Append the parking trajectories onto the start trajectories and build
                TrajectorySequenceBuilder preparedTraj = prepareBlueParkingTrajectories(parkingGen, trajState);
                TrajectorySequence parkingTraj = preparedTraj.build();
//                    TrajectorySequence traj = genTrajectory(trajState);

                // Put finished trajectory sequence into hashmap
                trajMap.put(trajState, new TrajectorySequence[]{posColorTraj, parkingTraj});

                // Since blue front and red back are identical and so are blue back and red front, we only generate
                // trajectories from the blue perspective and just reverse the color and start pos and insert
                //into the dictionary. Saves time and maintenace effort.
                TrajectoryState redReversedState = new TrajectoryState(
                        Color.RED,
                        startPos == StartPosition.BACK ? StartPosition.FRONT : StartPosition.BACK,
                        parkPos);
                trajMap.put(redReversedState, new TrajectorySequence[]{posColorTraj, parkingTraj});
//                    Log.d("TrajectoryGenerator", "Traj gen finished");
            }
        }
        return trajMap;
    }

    /**
     * Given a state, return the given start trajectories (the stuff before parking trajectories)
     * @param color The color of the current auto
     * @param startPos The start position of the current auto
     * @return The trajectory builder, with the start trajectories applied
     */
    @Deprecated
    private TrajectorySequenceBuilder prepareStartTrajectories(Color color, StartPosition startPos) {
        // Call separate methods for blue and red trajectories to avoid method clutter
        switch (color) {
            case RED:
                return prepareRedStartTrajectories(startPos);
            case BLUE:
                return prepareBlueStartTrajectories(startPos);
        }
        return null;
    }

    /**
     * Generate blue parking trajectories given start position
     * @param startPos The blue start position to generate trajectories for
     * @return The trajectory builder, with the start trajectories applied
     */
    private TrajectorySequenceBuilder prepareBlueStartTrajectories(StartPosition startPos) {
        // Do something different depending on start position
        switch (startPos) {
            case FRONT:
                // Front trajectories
                Pose2d startPose = new Pose2d(-40, 70 - (12.25 / 2.0), Math.toRadians(270));
                TrajectorySequenceBuilder gen = robot.drive.trajectorySequenceBuilder(startPose)
                        .addDisplacementMarker(() -> {
                            robot.rightClaw.setPosition(Constants.rightClawClosed);
                            robot.leftClaw.setPosition(Constants.leftClawClosed);
                        })
                        .strafeTo(new Vector2d(-12.5, 63.875))
                        .splineToConstantHeading(new Vector2d(-10, 57.1), Math.toRadians(270))
                        .addDisplacementMarker(() -> {
                            robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[2]);
                            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.linearSlide.setPower(1);
                        })
                        .splineToLinearHeading(new Pose2d(-2, 32, Math.toRadians(300)), Math.toRadians(300))

                        .splineToLinearHeading(new Pose2d(-1, 30, Math.toRadians(300)), Math.toRadians(300))
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
                        //.waitSeconds(1)
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
                            robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[2]);
                            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.linearSlide.setPower(1);
                        })

                        .splineToLinearHeading(new Pose2d(-28.5, 8.5, Math.toRadians(300)), Math.toRadians(300))

                        .splineToLinearHeading(new Pose2d(-25.5, 6, Math.toRadians(300)), Math.toRadians(300))
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
                        .splineToLinearHeading(new Pose2d(-35, 11, Math.toRadians(270)), Math.toRadians(270));

                return gen;
            case BACK:
                // Back trajectories. In progress right now.
                startPose = new Pose2d(29.5, 70 - (12.25 / 2.0), Math.toRadians(270));
                gen = robot.drive.trajectorySequenceBuilder(startPose)
                        .addDisplacementMarker(() -> {
                            robot.rightClaw.setPosition(Constants.rightClawClosed);
                            robot.leftClaw.setPosition(Constants.leftClawClosed);
                        })
                        .strafeTo(new Vector2d(12.5, 63.875))
                        .splineToConstantHeading(new Vector2d(10, 57.1), Math.toRadians(270))
                        .addDisplacementMarker(() -> {
                            robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[2]);
                            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.linearSlide.setPower(1);
                        })
                        .splineToLinearHeading(new Pose2d(2, 31, Math.toRadians(240)), Math.toRadians(240))

                        .splineToLinearHeading(new Pose2d(1, 29, Math.toRadians(240)), Math.toRadians(240))
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
                        .splineToConstantHeading(new Vector2d(56.5, 8), Math.toRadians(0))
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
//backing up from cone stack
                        .splineToLinearHeading(new Pose2d(27.5, 5.5, Math.toRadians(0)), Math.toRadians(0))
                        .addDisplacementMarker(() -> {
                            robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[2]);
                            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            robot.linearSlide.setPower(1);
                        })

                        .splineToLinearHeading(new Pose2d(28.5, 4.5, Math.toRadians(240)), Math.toRadians(240))

                        .splineToLinearHeading(new Pose2d(26, 2, Math.toRadians(240)), Math.toRadians(240))
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
                        .splineToLinearHeading(new Pose2d(35, 8, Math.toRadians(270)), Math.toRadians(270));
                return gen;
        }
        return null;
    }

    /**
     * Generate red parking trajectories given start position
     * @param startPos The red start position to generate trajectories for
     * @return The trajectory builder, with the start trajectories applied
     */
    @Deprecated
    private TrajectorySequenceBuilder prepareRedStartTrajectories(StartPosition startPos) {
        // This is all in progress.
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

    /**
     * Attach parking trajectories to a pre-existing trajectory builder. Expects that builder to
     * have start trajectories already.
     * @param gen The generator to use
     * @param trajState The state of the trajectory (color, start pos, park pos)
     * @return The builder, with parking trajectories attached
     */
    @Deprecated
    private TrajectorySequenceBuilder prepareParkingTrajectories(TrajectorySequenceBuilder gen, TrajectoryState trajState) {
        // Dispatch calls to separate methods by color, to reduce clutter
        switch (trajState.color) {
            case RED:
                return prepareRedParkingTrajectories(gen, trajState);
            case BLUE:
                return prepareBlueParkingTrajectories(gen, trajState);
        }
        return null;
    }

    /**
     * Attach parking trajectories to a pre-existing blue trajectory builder. Expects that builder to
     * have start trajectories already.
     * @param gen The blue generator to use
     * @param trajState The state of the trajectory (color, start pos, park pos)
     * @return The builder, with parking trajectories attached
     */
    private TrajectorySequenceBuilder prepareBlueParkingTrajectories(TrajectorySequenceBuilder gen, TrajectoryState trajState) {
        // Basically, just dispatch according to where we need to go. Not super complex.
        switch (trajState.startPosition) {
            case FRONT:
                switch (trajState.parkPos) {
                    case ONE:
                        gen.strafeTo(new Vector2d(-6, 12))
                                .waitSeconds(1);
                        break;
                    case TWO:
                        gen.waitSeconds(2);
                        break;
                    case THREE:
                        gen.strafeTo(new Vector2d(-60, 12))
                                .waitSeconds(1);
                        break;
                }
                break;
            case BACK:
                switch (trajState.parkPos) {
                    case ONE:
                        gen.strafeTo(new Vector2d(62, 8))
                                .waitSeconds(1);
                        break;
                    case TWO:
                        gen.waitSeconds(2);
                        break;
                    case THREE:
                        gen.strafeTo(new Vector2d(6, 8))
                                .waitSeconds(1);
                        break;
                }
                break;
        }
        return gen;
    }

    /**
     * Attach parking trajectories to a pre-existing red trajectory builder. Expects that builder to
     * have start trajectories already.
     * @param gen The red generator to use
     * @param trajState The state of the trajectory (color, start pos, park pos)
     * @return The builder, with parking trajectories attached
     */
    @Deprecated
    private TrajectorySequenceBuilder prepareRedParkingTrajectories(TrajectorySequenceBuilder gen, TrajectoryState trajState) {
        // Basically, just dispatch according to where we need to go. Not super complex.
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
                            robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[0]);
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
                            robot.linearSlide.setTargetPosition(Constants.getLiftEncoderConeStack()[0]);
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
                            robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[0]);
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

    /**
     * Internal class that represents all facets of autonomous (color, startPos, parkPos)
     */
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
