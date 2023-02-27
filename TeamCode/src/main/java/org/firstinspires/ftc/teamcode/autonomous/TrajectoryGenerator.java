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
    // Various useful variables
    private final Hardware23 robot;
    private final SampleMecanumDrive drive;
    private final Telemetry telemetry;
    private Vector2d offset = new Vector2d(0, 0);

    /*
      Stores mapping between Auto State (color, start pos, park pos) and traj sequence to run.
      Stores an array of start trajectory and parking trajectory for efficiency reasons. It is inefficient to
      regenerate start traj for each parking position, but we can't use the same start traj for each parking
      trajectory because there is no good way to copy a TrajectorySequenceBuilder. As a result, we generate start
      and parking trajectories separately, and store them in an array in this dictionary. It's annoying, but it's what
      we gotta do.
     */
    private HashMap<TrajectoryState, TrajectorySequence[]> trajectories;

    /**
     * Contructor. Calls a method to generate only appropriate trajectories for current auto state
     * @param robot The hardware file for the robot
     * @param autoState The trajectories to generate
     * @param offset The start offset
     * @param telemetry The telemetry object
     */
    public TrajectoryGenerator(Hardware23 robot, AutoState autoState, Vector2d offset, Telemetry telemetry) {
        this.robot = robot;
        this.drive = robot.drive;
        this.telemetry = telemetry;
        this.offset = offset;

        // Self explanatory. Populates trajectories variable
        this.trajectories = generateSpecificTrajectories(autoState);
    }

    private HashMap<TrajectoryState, TrajectorySequence[]> generateSpecificTrajectories(AutoState autoState) {
        HashMap<TrajectoryState, TrajectorySequence[]> trajMap = new HashMap<>();

        StartPosition startPos;
        // Invert start position if color is red
        if (autoState.color == Color.RED) {
            startPos = autoState.position == StartPosition.FRONT ? StartPosition.BACK : StartPosition.FRONT;
        } else {
            startPos = autoState.position;
        }

        autoState = new AutoState(autoState.color, startPos, autoState.autoType, autoState.delay);

        // Log to the console to tell user what we are working on
        Log.d("TrajectoryGenerator", String.format("Generating start traj - %s %s", startPos, autoState.autoType));
        telemetry.addLine(String.format("Generating start traj - %s %s", startPos, autoState.autoType));
        telemetry.update();

        // Since the first part of each trajectory (the stuff before parking) is the same,
        // we generate it separately and don't build to avoid wasting a lot of time.
        TrajectorySequence posColorTraj = prepareBlueStartTrajectories(autoState).build();

        // Loop through each parking position
        for (ParkingPosition parkPos : new ParkingPosition[]{ParkingPosition.ONE, ParkingPosition.TWO, ParkingPosition.THREE}) {
            // Have to make this generator here or we do all parking trajectories no matter what
            TrajectorySequenceBuilder parkingGen = drive.trajectorySequenceBuilder(posColorTraj.end());
            // Once again, make sure to log
            Log.d("TrajectoryGenerator", String.format("Generating parking traj - %s %s %s", startPos, autoState.autoType, parkPos));
            telemetry.addLine(String.format("Generating parking traj - %s %s %s", startPos, autoState.autoType, parkPos));
            telemetry.update();

            // Construct this to pass into parking trajectories method
            TrajectoryState trajState = new TrajectoryState(Color.BLUE, startPos, parkPos, autoState.autoType);

            // Append the parking trajectories onto the start trajectories and build
            TrajectorySequenceBuilder preparedTraj = prepareBlueParkingTrajectories(parkingGen, trajState);
            TrajectorySequence parkingTraj = preparedTraj.build();
//                    TrajectorySequence traj = genTrajectory(trajState);

            // Put finished trajectory sequence into hashmap
            trajMap.put(trajState, new TrajectorySequence[]{posColorTraj, parkingTraj});
//                    Log.d("TrajectoryGenerator", "Traj gen finished");
        }
        return trajMap;
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
        TrajectoryState tempTrajState = new TrajectoryState(autoState.color, autoState.position, parkPos, autoState.autoType);
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
     * Generate blue parking trajectories given auto state
     * @param autoState The state of the autonomous to generate trajectories for. Ignores color
     * @return The trajectory builder, with the start trajectories applied
     */
    private TrajectorySequenceBuilder prepareBlueStartTrajectories(AutoState autoState) {
        // Do something different depending on start position
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
    private TrajectorySequenceBuilder getSweatFrontTrajectories() {
        throw new NotImplementedError("Moron were not done with this");
    }

    // TODO work on this
    private TrajectorySequenceBuilder getSweatBackTrajectories() {
        throw new NotImplementedError("Moron were not done with this");
    }

    private TrajectorySequenceBuilder getLongFrontTrajectories() {
        // Front trajectories
        Pose2d startPose = new Pose2d(-40, 70 - (12.25 / 2.0), Math.toRadians(270));
        // To handle offsetting
        startPose.plus(new Pose2d(offset.getX(), offset.getY()));

        TrajectorySequenceBuilder gen = robot.drive.trajectorySequenceBuilder(startPose)
//                        .setTurnConstraint(60, 0.5)
                .addDisplacementMarker(() -> {
                    robot.rightClaw.setPosition(Constants.rightClawClosed);
                    robot.leftClaw.setPosition(Constants.leftClawClosed);
                })
                .strafeTo(new Vector2d(-12.5, 60))
                .splineToConstantHeading(new Vector2d(-10, 57.1), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[2]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .splineToLinearHeading(new Pose2d(-2, 32, Math.toRadians(300)), Math.toRadians(300))

                .splineToLinearHeading(new Pose2d(-.5, 30, Math.toRadians(300)), Math.toRadians(300))
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
                .splineToConstantHeading(new Vector2d(-56.5, 8), Math.toRadians(180))
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
                // backing up from cone stack
                .splineToLinearHeading(new Pose2d(-30, 11, Math.toRadians(180)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[1]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })

//                        .setTurnConstraint(60, 2)
//                        .turn(Math.toRadians(-120))

                .splineToLinearHeading(new Pose2d(-32, 11, Math.toRadians(50)), Math.toRadians(50))
                .splineToLinearHeading(new Pose2d(-26.5, 13.25, Math.toRadians(50)), Math.toRadians(50))
                .forward(4)
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
                .splineToLinearHeading(new Pose2d(-35, 11, Math.toRadians(90)), Math.toRadians(90));

        return gen;
    }

    private TrajectorySequenceBuilder getLongBackTrajectories() {
        TrajectorySequenceBuilder gen;
        Pose2d startPose = new Pose2d(29.5, 70 - (12.25 / 2.0), Math.toRadians(270));
        startPose.plus(new Pose2d(offset.getX(), offset.getY()));

        gen = robot.drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    robot.rightClaw.setPosition(Constants.rightClawClosed);
                    robot.leftClaw.setPosition(Constants.leftClawClosed);
                })
                .strafeTo(new Vector2d(12.5, 60))
                .splineToConstantHeading(new Vector2d(10, 57.1), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[2]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                .splineToLinearHeading(new Pose2d(1, 32, Math.toRadians(240)), Math.toRadians(240))

                .splineToLinearHeading(new Pose2d(0, 30.5, Math.toRadians(240)), Math.toRadians(240))
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
                .splineToLinearHeading(new Pose2d(11.5, 8.5, Math.toRadians(0)), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderConeStack()[0]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })
                //approach cone stack
                .splineToConstantHeading(new Vector2d(56, 8), Math.toRadians(0))
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
                .splineToLinearHeading(new Pose2d(27.5, 5.5, Math.toRadians(0)), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[1]);
                    robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.linearSlide.setPower(1);
                })

                .splineToLinearHeading(new Pose2d(27.5, 9, Math.toRadians(120)), Math.toRadians(210))

                .splineToLinearHeading(new Pose2d(24.5, 13.5, Math.toRadians(120)), Math.toRadians(120))
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
                .splineToLinearHeading(new Pose2d(35, 6, Math.toRadians(90)), Math.toRadians(90));
        return gen;
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
        switch (trajState.autoType) {
            case LONG:
                return getLongParkingTrajectories(gen, trajState);
            case SWEAT:
                return getSweatParkingTrajectories(gen, trajState);
        }
        return gen;
    }

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
     * Internal class that represents all facets of autonomous (color, startPos, parkPos)
     */
    public static class TrajectoryState {
        public final Color color;
        public final StartPosition startPosition;
        public final ParkingPosition parkPos;
        public final AutoType autoType;

        TrajectoryState() {
            this.color = null;
            this.startPosition = null;
            this.parkPos = null;
            this.autoType = null;
        }

        TrajectoryState(Color color, StartPosition startPosition, ParkingPosition parkPos, AutoType autoType) {
            this.color = color;
            this.startPosition = startPosition;
            this.parkPos = parkPos;
            this.autoType = autoType;
        }

        public boolean hasSameValue(TrajectoryState state) {
            return this.color == state.color && this.startPosition == state.startPosition && this.parkPos == state.parkPos;
        }
    }
}
