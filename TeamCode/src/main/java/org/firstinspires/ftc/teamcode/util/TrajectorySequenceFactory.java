package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.PathContinuityViolationException;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import java.util.ArrayList;

import kotlin.NotImplementedError;

/**
 * This class is a wrapper for the trajectory sequence builder that streamlines the process to make
 * it (hopefully) a little bit easier. Initialize an instance and call .navigateTo methods in
 * a chain to navigate either to absolute positions or with vector position changes.
 */
public class TrajectorySequenceFactory {
    // Splines take this as an argument, so we have this for default value methods.
    private Double endTangentDefault = 0d;

    // We internally store the path as a list of double arrays, to generate later.
    private final ArrayList<Double[]> internalPathRepresentation;

    private final SampleMecanumDrive drive;
    // We only need this to warn the user if the path is not continuous when building.
    private final Telemetry telemetry;
    private final Pose2d startPose;

    /**
     * Default constructor. Takes essential variables needed to properly generate paths.
     * @param drive The SampleMecanumDrive that the robot uses to drive. Used to get trajectoryBuilders.
     * @param telemetry The telemetry object of the opmode. Used only to warn the user of path discontinuity.
     * @param startPose The starting pose. Self explanatory.
     */
    public TrajectorySequenceFactory(SampleMecanumDrive drive, Telemetry telemetry, Pose2d startPose) {
        // The Double arrays here have the following structure:
        //  [x, y, heading, pathType, endTangent (will be -999 if not applicable)]
        this.internalPathRepresentation= new ArrayList<>();
        this.drive = drive;
        this.telemetry = telemetry;
        this.startPose = startPose;

        // If first navigation is a delta instead of a pose, we need this as a starting reference
        internalPathRepresentation.add(new Double[]{startPose.getX(), startPose.getY(), startPose.getHeading(), -1d, -999d});
    }

    /**
     * Default value absolute position navigation method. Will spline to given pose with
     * spline heading interpolation at default end tangent.
     * @param pose The destination pose
     * @return This same object, to allow for method chaining
     */
    public TrajectorySequenceFactory navigateTo(Pose2d pose) {
        return this.navigateTo(pose, PathType.SPLINE_TO_SPLINE, endTangentDefault);
    }

    /**
     * Default value relative position navigation method. Will spline to calculated destination.
     * @param delta A vector with the change from the current position.
     * @return This same object, to allow for method chaining
     */
    public TrajectorySequenceFactory navigateTo(Vector2d delta) {
        return this.navigateTo(delta, PathType.SPLINE_TO_SPLINE);
    }

    /**
     * Absolute position navigation method. Will navigate to given pose with given path type.
     * @param pose The destination pose
     * @param pathType The path type to use, given by PathType enum
     * @param endTangent For spline path types, spline end tangent. Ignored if not spline
     *                   path type.
     * @return This same object, to allow for method chaining
     */
    public TrajectorySequenceFactory navigateTo(Pose2d pose, PathType pathType, Double endTangent) {
        internalPathRepresentation.add(
                new Double[] {
                        pose.getX(),
                        pose.getY(),
                        pose.getHeading(),
                        (double) pathType.getValue(),

                        // If the pathType is a spline movement type (NOT spline heading), then we need to
                        //use end tangent, otherwise we need to put -999 as that
                        // signifies that there is no endTangent. Technically, this is not strictly required,
                        // but it helps with readability.
                        pathType == PathType.SPLINE_TO_SPLINE || pathType == PathType.SPLINE_TO_LINEAR ? endTangent : -999d}
        );
        return this;
    }

    /**
     * Relative poisition navigation method. Navigates to change from current position given by
     * delta using given path type.
     * @param delta The vector position change from current position
     * @param pathType The path type to follow
     * @return This same object, to allow for method chaining
     */
    public TrajectorySequenceFactory navigateTo(Vector2d delta, PathType pathType) {
        return this.navigateTo(
                new Pose2d(delta.getX(),
                        delta.getY(),
                        getLatestPose().getHeading()),
                pathType,
                // If the pathType is a spline movement type (NOT spline heading), then we need to
                // pass the default end tangent into the method, otherwise we need to put -999 as that
                // signifies that there is no endTangent. Technically, this is not strictly required,
                // but it helps with readability.
                pathType == PathType.SPLINE_TO_SPLINE || pathType == PathType.SPLINE_TO_LINEAR ? endTangentDefault : -999d
        );
    }

    /**
     * Build method. Returns generated TrajectorySequence to follow.
     * @return The TrajectorySequence to follow
     */
    public TrajectorySequence build() {
        TrajectorySequenceBuilder builder = null;
        TrajectorySequence sequence = null;
        Trajectory traj = null;

        // Test for path continuity. If the path is not continuous, it throws an error that we can
        // then catch and use to warn the user. If the trajectory generates successfully, we can use
        // that instead of regenerating the trajectory sequence, otherwise we need to do it from
        // scratch.
        try {
            TrajectoryBuilder builderTest = null;

            for (Double[] posArr : internalPathRepresentation) {
                Pose2d currPose = composePoseFromArr(posArr);
                switch ((int) Math.round(posArr[3])) {
                    case -1:
                        builderTest = drive.trajectoryBuilder(startPose);
                        break;
                    case 0:
                        builderTest.lineToLinearHeading(currPose);
                        break;
                    case 1:
                        builderTest.lineToSplineHeading(currPose);
                        break;
                    case 2:
                        builderTest.splineToLinearHeading(currPose, posArr[4]);
                        break;
                    case 3:
                        builderTest.splineToSplineHeading(currPose, posArr[4]);
                        break;
                    default:
                        throw new IllegalStateException("Unexpected path type: " + (int) Math.round(posArr[3]));
                }
                traj = builderTest.build();
            }
        } catch (PathContinuityViolationException e) {
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
            telemetry.addLine("<p style='color: #daa052;'>WARNING. Path is not continuous.</p>");
            telemetry.update();
            traj = null;
//         throw e;
        }
        // If trajectory was not initialized properly, need to regenerate traj sequence, otherwise we
        // don't need to do it and can just add the traj to the traj sequence.
        // There is probably a cleaner, faster, and more efficient way to do this, but I don't care.
        // It should work.
        if (traj == null) {
            for (Double[] posArr : internalPathRepresentation) {
                Pose2d currPose = composePoseFromArr(posArr);
                switch ((int) Math.round(posArr[3])) {
                    case -1:
                        builder = drive.trajectorySequenceBuilder(startPose);
                        break;
                    case 0:
                        builder.lineToLinearHeading(currPose);
                        break;
                    case 1:
                        builder.lineToSplineHeading(currPose);
                        break;
                    case 2:
                        builder.splineToLinearHeading(currPose, posArr[4]);
                        break;
                    case 3:
                        builder.splineToSplineHeading(currPose, posArr[4]);
                        break;
                    default:
                        throw new IllegalStateException("Unexpected path type: " + (int) Math.round(posArr[3]));
                }
                sequence = builder.build();
            }
            // If the trajectory from earlier generated properly, can just add it to the sequence.
        } else {
            sequence = drive.trajectorySequenceBuilder(composePoseFromArr(internalPathRepresentation.get(0)))
                    .addTrajectory(traj)
                    .build();
        }
        return sequence;
    }

    /**
     * Utility method. Fetches last pose in list.
     * @return The last pose in the internal list.
     */
    private Pose2d getLatestPose() {
        Double[] lastArr = internalPathRepresentation.get(internalPathRepresentation.size() - 1);
        return composePoseFromArr(lastArr);
    }

    /**
     * Utility method. Converts double array to Pose2D.
     * @param posArr The double array to be converted
     * @return The converted Pose2D
     */
    private Pose2d composePoseFromArr(Double[] posArr) {
        return new Pose2d(posArr[0], posArr[1], posArr[2]);
    }

    /**
     * Sets default end tangent to use for spline paths.
     * @param endTangentDefault The default end tangent to use
     */
    public void setDefaultEndTangent(double endTangentDefault) {
        this.endTangentDefault = endTangentDefault;
    }
}
