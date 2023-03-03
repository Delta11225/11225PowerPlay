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

import kotlin.NotImplementedError;

public class TrajectoryGenerator {
    private final Hardware23 robot;
    private final SampleMecanumDrive drive;

    // The telemetry object we send telemetry to
    private final Telemetry telemetry;

    // Stores start position offset provided by user
    private final Vector2d offset;

    /*
    Stores a mapping between parking position and trajectory sequences, as we don't know parking
    position before auto starts, so we generate all possible trajectories for the given autostate
    beforehand and store them in this hashmap
     */
    private final HashMap<ParkingPosition, TrajectorySequence[]> trajectories;

    /**
     * Contructor. Calls a method to generate only appropriate trajectories for current auto state
     * @param robot The hardware file for the robot
     * @param autoState The trajectories to generate
     * @param offset The start offset. Basically by how much all start positions should be adjusted
     * @param telemetry The telemetry object
     */
    public TrajectoryGenerator(Hardware23 robot, AutoState autoState, Vector2d offset, Telemetry telemetry) {
        this.robot = robot;
        this.drive = robot.drive;
        this.telemetry = telemetry;
        this.offset = offset;

        // Populates trajectories variable with generated trajectories
        this.trajectories = generateAppropriateTrajectories(autoState);
    }

    /**
     * Generates trajectories based off of provided autostate. Also generates all parking trajectories.
     * @param autoState The autostate to generate trajectories by
     * @return A HashMap between parking position and appropriate list of trajectories
     */
    private HashMap<ParkingPosition, TrajectorySequence[]> generateAppropriateTrajectories(AutoState autoState) {
        HashMap<ParkingPosition, TrajectorySequence[]> trajMap = new HashMap<>();

        StartPosition startPos;
        // Invert start position if color is red
        if (autoState.color == Color.RED) {
            startPos = autoState.position == StartPosition.FRONT ? StartPosition.BACK : StartPosition.FRONT;
        } else {
            startPos = autoState.position;
        }

        // Create the new autostate based on selected color
        autoState = new AutoState(autoState.color, startPos, autoState.autoType, autoState.delay);

        // Log to tell user what we are working on
        Log.d("TrajectoryGenerator", String.format("Generating start traj - %s %s", startPos, autoState.autoType));
        telemetry.addLine(String.format("Generating start traj - %s %s", startPos, autoState.autoType));
        telemetry.update();

        // Since the first part of each trajectory (the stuff before parking) is the same,
        // we generate it separately from parking trajectories
        TrajectorySequence posColorTraj = prepareBlueStartTrajectories(autoState).build();

        // Loop through each parking position and generate appropriate parking trajectory
        for (ParkingPosition parkPos : new ParkingPosition[]{ParkingPosition.ONE, ParkingPosition.TWO, ParkingPosition.THREE}) {
            // Once again, make sure to log
            Log.d("TrajectoryGenerator", String.format("Generating parking traj - %s %s %s", startPos, autoState.autoType, parkPos));
            telemetry.addLine(String.format("Generating parking traj - %s %s %s", startPos, autoState.autoType, parkPos));
            telemetry.update();

            // Construct this to pass into parking trajectories method
            TrajectoryState trajState = new TrajectoryState(startPos, parkPos, autoState.autoType);

            // Make a generator that starts at where the previous parking trajectory ends
            TrajectorySequenceBuilder parkingGen = drive.trajectorySequenceBuilder(posColorTraj.end());
            // Get and build parking trajectories
            TrajectorySequence parkingTraj = prepareBlueParkingTrajectories(parkingGen, trajState)
                    .build();

            // Put finished trajectories (start and parking) into hashmap along with appropriate parking
            // trajectory
            trajMap.put(trajState.parkPos, new TrajectorySequence[]{posColorTraj, parkingTraj});
//                    Log.d("TrajectoryGenerator", "Traj gen finished");
        }
        return trajMap;
    }

    /**
     * Return the trajectory sequence corresponding with the parking position given
     * @param parkPos The required parking position
     * @return The appropriate trajectory sequence for this auto state
     */
    public TrajectorySequence[] getAppropriateTrajectory(ParkingPosition parkPos) {
        // We only need to take parkpos as we don't generate all possible trajectories all at once
        Log.d("TrajectoryGenerator", String.format("Requested trajectory - %s", parkPos));

        // Fetch the appropriate trajectories from the hashmap
        TrajectorySequence[] traj = trajectories.getOrDefault(parkPos, null);
        
        // If traj is null, it means it wasn't in our hashmap, and something has gone wrong. Let the user know.
        if (traj == null) {
            // Throw and error and report the error to the log (debug reasons)
            Log.e("TrajectoryGenerator", "WTF? How? You somehow have provide an auto state that there" +
                    "aren't trajectories for.");
            throw new IllegalStateException("WTF? How? You somehow have provide an auto state that there" +
                    "aren't trajectories for.");
        }

        // Make sure to tell the robot where it is based on where the trajectory starts
        drive.setPoseEstimate(traj[0].start());
        return traj;
    }

    /**
     * Generate blue parking trajectories given auto state
     * @param autoState The state of the autonomous to generate trajectories for. Ignores color
     * @return The trajectory builder, with the start trajectories applied
     */
    private TrajectorySequenceBuilder prepareBlueStartTrajectories(AutoState autoState) {
        // Do something different depending on auto type and start position
        switch (autoState.autoType) {
            case LONG:
                switch (autoState.position) {
                    case FRONT:
                        return getLongFrontTrajectories();
                    case BACK:
                        return getLongBackTrajectories();
                };
            case SWEAT:
                switch (autoState.position) {
                    case FRONT:
                        return getSweatFrontTrajectories();
                    case BACK:
                        return getSweatBackTrajectories();
                }
        }
        return null;
    }

    // TODO work on this
    /**
     * Get front position trajectories for sweat autonomous type
     * @return A builder for the sweat front trajectories
     */
    private TrajectorySequenceBuilder getSweatFrontTrajectories() {
        throw new NotImplementedError("Moron we're not done with this");
    }

    // TODO work on this
    /**
     * Get back position trajectories for sweat autonomous type
     * @return A builder for the sweat back trajectories
     */
    private TrajectorySequenceBuilder getSweatBackTrajectories() {
        throw new NotImplementedError("Moron we're not done with this");
    }

    /**
     * Get the start trajectories for the long auto and the front starting position
     * @return
     */
    private TrajectorySequenceBuilder getLongFrontTrajectories() {
        // Where the robot starts
        Pose2d startPose = new Pose2d(-40, 70 - (12.25 / 2.0), Math.toRadians(270));
        // Add offset to start pose
        startPose.plus(new Pose2d(offset.getX(), offset.getY()));

        // We return a generator instead of the trajectory in case we want to add to it
        TrajectorySequenceBuilder gen = robot.drive.trajectorySequenceBuilder(startPose)
//                        .setTurnConstraint(60, 0.5)
                // Close the claw at start
                .addDisplacementMarker(() -> {
                    robot.rightClaw.setPosition(Constants.rightClawClosed);
                    robot.leftClaw.setPosition(Constants.leftClawClosed);
                })
                // Move to side and up a bit
                .strafeTo(new Vector2d(-12.5, 60))
                .splineToConstantHeading(new Vector2d(-10, 57.1), Math.toRadians(270))
                // Lift up linear slide
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[2]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                // Go to middle high junction
                .splineToLinearHeading(new Pose2d(-2, 32, Math.toRadians(300)), Math.toRadians(300))
                // Approach it a bit closer
                .splineToLinearHeading(new Pose2d(-.5, 30, Math.toRadians(300)), Math.toRadians(300))
                // Drop cone
                .addDisplacementMarker(() -> {
                    robot.leftClaw.setPosition(Constants.leftClawOpen);
                    robot.rightClaw.setPosition(Constants.rightClawOpen);
                })
                // Wait for cone to fall
                .waitSeconds(0.25)
                // Back up and lower slide
                .splineToLinearHeading(new Pose2d(-6, 36, Math.toRadians(300)), Math.toRadians(300))
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(0);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                // Square up with cone stack
                .splineToLinearHeading(new Pose2d(-10, 30, Math.toRadians(270)), Math.toRadians(270))
                // Turn to face cone stack and lift linear slide to appropriate height
                .splineToLinearHeading(new Pose2d(-12, 11, Math.toRadians(180)), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderConeStack()[0]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                // Approach cone stack and wait a second
                .splineToConstantHeading(new Vector2d(-56.5, 8), Math.toRadians(180))
                .waitSeconds(0.25)

                // Grab cone
                .addDisplacementMarker(() -> {
                    robot.rightClaw.setPosition(Constants.rightClawClosed);
                    robot.leftClaw.setPosition(Constants.leftClawClosed);
                })
                // This tiny forward forces the robot to actually wait after the displacement
                // marker, as for some reason it won't otherwise. Hacky? Yes. Functional? Also yes.
                .forward(0.001)
                .waitSeconds(0.25)
                // Go to low junction height to lift off cone stack
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[0]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                // Back up from cone stack
                .back(5)
                .splineToLinearHeading(new Pose2d(-30, 11, Math.toRadians(180)), Math.toRadians(0))
                // Go to medium junction height
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[1]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                // Get to medium junction
                .splineToLinearHeading(new Pose2d(-32, 11, Math.toRadians(50)), Math.toRadians(50))
                .splineToLinearHeading(new Pose2d(-26.5, 13.25, Math.toRadians(50)), Math.toRadians(50))
                .forward(4)
                // Drop cone
                .addDisplacementMarker(() -> {
                    robot.rightClaw.setPosition(Constants.rightClawOpen);
                    robot.leftClaw.setPosition(Constants.leftClawOpen);
                })
                .waitSeconds(0.25)
                // Back up from medium
                .back(5)
                // Go to ground
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(0);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                // Go to position to prepare for parking
                .splineToLinearHeading(new Pose2d(-35, 11, Math.toRadians(90)), Math.toRadians(90));

        return gen;
    }

    /**
     * Get the start trajectories for the long auto and the back starting position
     * @return
     */
    private TrajectorySequenceBuilder getLongBackTrajectories() {
        // Where the robot starts
        Pose2d startPose = new Pose2d(29.5, 70 - (12.25 / 2.0), Math.toRadians(270));
        // Add offset to start pose
        startPose.plus(new Pose2d(offset.getX(), offset.getY()));

        // We return a generator instead of the trajectory in case we want to add to it
        TrajectorySequenceBuilder gen = robot.drive.trajectorySequenceBuilder(startPose)
                // Close the claw at start
                .addDisplacementMarker(() -> {
                    robot.rightClaw.setPosition(Constants.rightClawClosed);
                    robot.leftClaw.setPosition(Constants.leftClawClosed);
                })
                // Move to side and up a bit
                .strafeTo(new Vector2d(12.5, 60))
                .splineToConstantHeading(new Vector2d(10, 57.1), Math.toRadians(270))
                // Lift up linear slide
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[2]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                // Go to middle high junction
                .splineToLinearHeading(new Pose2d(1, 32, Math.toRadians(240)), Math.toRadians(240))
                // Approach it a bit closer
                .splineToLinearHeading(new Pose2d(0, 30.5, Math.toRadians(240)), Math.toRadians(240))
                // Drop cone
                .addDisplacementMarker(() -> {
                    robot.leftClaw.setPosition(Constants.leftClawOpen);
                    robot.rightClaw.setPosition(Constants.rightClawOpen);
                })
                // Wait for cone to fall
                .waitSeconds(0.25)
                // Back up and lower slide
                .splineToLinearHeading(new Pose2d(6, 36, Math.toRadians(240)), Math.toRadians(240))
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(0);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })

                // Square up with cone stack
                .splineToLinearHeading(new Pose2d(10, 30, Math.toRadians(270)), Math.toRadians(270))
                // Turn to face cone stack and lift linear slide to appropriate height
                .splineToLinearHeading(new Pose2d(11.5, 8.5, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderConeStack()[0]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                // Approach cone stack and wait a second
                .splineToConstantHeading(new Vector2d(56, 8), Math.toRadians(0))
                .waitSeconds(0.25)

                // Grab cone
                .addDisplacementMarker(() -> {
                    robot.rightClaw.setPosition(Constants.rightClawClosed);
                    robot.leftClaw.setPosition(Constants.leftClawClosed);
                })
                // This tiny forward forces the robot to actually wait after the displacement
                // marker, as for some reason it won't otherwise. Hacky? Yes. Functional? Also yes.
                .forward(0.001)
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[0]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                // Back up from cone stack
                .back(5)
                .splineToLinearHeading(new Pose2d(27.5, 5.5, Math.toRadians(0)), Math.toRadians(180))
                // Go to medium junction height
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[1]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                // Get to medium junction
                .splineToLinearHeading(new Pose2d(27.5, 9, Math.toRadians(120)), Math.toRadians(210))

                // Drop cone
                .splineToLinearHeading(new Pose2d(24.5, 13.5, Math.toRadians(120)), Math.toRadians(120))
                .addDisplacementMarker(() -> {
                    robot.rightClaw.setPosition(Constants.rightClawOpen);
                    robot.leftClaw.setPosition(Constants.leftClawOpen);
                })
                .waitSeconds(0.25)
                // Back up from medium
                .back(5)
                // Go to ground
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(0);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                // Go to position to prepare for parking
                .splineToLinearHeading(new Pose2d(35, 6, Math.toRadians(90)), Math.toRadians(90));
        return gen;
    }

    /**
     * Generate parking trajectories. Expects an initialized trajectory builder
     * @param gen The blue generator to use
     * @param trajState The state of the trajectory (color, start pos, park pos)
     * @return The builder, with parking trajectories attached
     */
    private TrajectorySequenceBuilder prepareBlueParkingTrajectories(TrajectorySequenceBuilder gen, TrajectoryState trajState) {
        // Basically, just dispatch according to where we need to go. Not super complex.
        switch (trajState.autoType) {
            case LONG:
                return getLongParkingTrajectories(gen, trajState);
            case SWEAT:
                return getSweatParkingTrajectories(gen, trajState);
        }
        return gen;
    }

    /**
     * Generate parking trajectories for long autonomous. Expects an initialized trajectory generator
     * @param gen The initialized trajectory generator
     * @param trajState The state for which to generate parking trajectories for
     * @return A builder with parking trajectories attached
     */
    private TrajectorySequenceBuilder getLongParkingTrajectories(TrajectorySequenceBuilder gen, TrajectoryState trajState) {
        switch (trajState.startPosition) {
            case FRONT:
                switch (trajState.parkPos) {
                    case ONE:
                        gen.strafeTo(new Vector2d(-6, 7))
                                .waitSeconds(1);
                        break;
                    case TWO:
                        gen.waitSeconds(2);
                        break;
                    case THREE:
                        gen.strafeTo(new Vector2d(-60, 7))
                                .waitSeconds(1);
                        break;
                }
                break;
            case BACK:
                switch (trajState.parkPos) {
                    case ONE:
                        gen.strafeTo(new Vector2d(60, 7))
                                .waitSeconds(1);
                        break;
                    case TWO:
                        gen.waitSeconds(2);
                        break;
                    case THREE:
                        gen.strafeTo(new Vector2d(6, 7))
                                .waitSeconds(1);
                        break;
                }
                break;
        }
        return gen;
    }

    // TODO work on this for parking
    /**
     * Generate parking trajectories for sweat autonomous. Expects an initialized trajectory generator
     * @param gen The initialized trajectory generator
     * @param trajState The state for which to generate parking trajectories for
     * @return A builder with parking trajectories attached
     */
    private TrajectorySequenceBuilder getSweatParkingTrajectories(TrajectorySequenceBuilder gen, TrajectoryState trajState) {
        switch (trajState.startPosition) {
            case FRONT:
                switch (trajState.parkPos) {
                    case ONE:
                        break;
                    case TWO:
                        break;
                    case THREE:
                        break;
                }
                break;
            case BACK:
                switch (trajState.parkPos) {
                    case ONE:
                        break;
                    case TWO:
                        break;
                    case THREE:
                        break;
                }
                break;
        }
        return gen;
    }

    /**
     * Internal class that represents all facets of a trajectory, sans color since it doesn't matter
     * (startPos, parkPos, and autoType)
     */
    public static class TrajectoryState {
        public final StartPosition startPosition;
        public final ParkingPosition parkPos;
        public final AutoType autoType;

        TrajectoryState(StartPosition startPosition, ParkingPosition parkPos, AutoType autoType) {
            this.startPosition = startPosition;
            this.parkPos = parkPos;
            this.autoType = autoType;
        }

        public boolean hasSameValue(TrajectoryState state) {
            return this.startPosition == state.startPosition && this.parkPos == state.parkPos;
        }
    }
}
