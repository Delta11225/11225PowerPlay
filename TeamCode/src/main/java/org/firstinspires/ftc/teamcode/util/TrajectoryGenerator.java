package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;

public class TrajectoryGenerator {
    SampleMecanumDrive drive;

    public TrajectoryGenerator(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    /**
     * @param x            - The x amount the trajectory should go
     * @param y            - The y amount the trajectory should go
     * @param heading      - The heading in degrees that the robot should end up at (if applicable)
     * @param endTangent
     * @param pathType     - The type of path used
     * @return - An integer array that can be appended to a trajectory path and executed
     */
    public Double[] generateTrajectoryListItem(double x, double y, double heading, double endTangent, PathType pathType, ArrayList<Double[]> arr) {
        if (pathType != PathType.SPLINE_TO_LINEAR) {
            throw new IllegalArgumentException("For this generator, path type must be splineToLinear");
        }

        double pathDouble = pathType.getValue();
        Double[] doubleArr = new Double[]{x, y, heading, endTangent, pathDouble};
        arr.add(doubleArr);
        return doubleArr;
    }

    public Double[] generateTrajectoryListItem(double x, double y, double heading, PathType pathType, ArrayList<Double[]> arr) {
        if (pathType != PathType.LINE_TO_LINEAR) {
            throw new IllegalArgumentException("For this generator, path type must be lineToLinear");
        }

        double pathDouble = pathType.getValue();
        Double[] doubleArr = new Double[]{x, y, heading, -1.0, pathDouble};
        arr.add(doubleArr);
        return doubleArr;
    }

    public Double[] generateTrajectoryListItem(double x, double y, PathType pathType, ArrayList<Double[]> arr) {
        if (pathType != PathType.LINE_TO_CONSTANT) {
            throw new IllegalArgumentException("For this generator, path type must be lineToConstant");
        }

        double pathDouble = pathType.getValue();
        Double[] doubleArr = new Double[]{x, y, -1.0, -1.0, pathDouble};
        arr.add(doubleArr);
        return doubleArr;
    }

    // Execute a list of trajectories as compiled by compileTrajectoryList
    public static Pose2d executeTrajectoryList(SampleMecanumDrive drive, ArrayList<Trajectory> arr) {
        for (Trajectory item : arr) {
            drive.followTrajectory(item);
        }
        return arr.get(arr.size() - 1).end();
    }

    // TODO this should not be used in favor of compileTrajectory as that one uses proper standards
    // TODO and is faster.
    // TODO remove all references to this method and replace with compileTrajectory
//     @Deprecated
    public ArrayList<Trajectory> compileTrajectoryList(Pose2d start, ArrayList<Double[]> arr) {
        ArrayList<Trajectory> compiled = new ArrayList<>();
        Pose2d previous = start;
        for (Double[] item : arr) {
            PathType type = PathType.valueOf(item[4].intValue());
            Trajectory traj;
            switch (type) {
                case LINE_TO_CONSTANT:
                    traj = drive.trajectoryBuilder(previous, true)
                            .lineToConstantHeading(new Vector2d(item[0], item[1]))
                            .build();
                    break;
                case LINE_TO_LINEAR:
                    traj = drive.trajectoryBuilder(previous, true)
                            .lineToLinearHeading(new Pose2d(item[0], item[1], Math.toRadians(item[2])))
                            .build();
                    break;
                case SPLINE_TO_LINEAR:
                    traj = drive.trajectoryBuilder(previous, true)
                            .splineToLinearHeading(new Pose2d(item[0], item[1], Math.toRadians(item[2])), Math.toRadians(item[3]))
                            .build();
                    break;
                default:
                    throw new IllegalStateException("Unexpected value: " + type);
            }
            compiled.add(traj);
            previous = traj.end();
        }
        return compiled;
    }

    public Trajectory compileTrajectory(Pose2d start, ArrayList<Double[]> arr) {
        TrajectoryBuilder builder = drive.trajectoryBuilder(start, true);
        for (Double[] item : arr) {
            PathType type = PathType.valueOf(item[4].intValue());
            switch (type) {
                case LINE_TO_CONSTANT:
                    builder.lineToConstantHeading(new Vector2d(item[0], item[1]));
                    break;
                case LINE_TO_LINEAR:
                    builder.lineToLinearHeading(new Pose2d(item[0], item[1], Math.toRadians(item[2])));
                    break;
                case SPLINE_TO_LINEAR:
                    builder.splineToLinearHeading(new Pose2d(item[0], item[1], Math.toRadians(item[2])), Math.toRadians(item[3]));
                    break;
                default:
                    throw new IllegalStateException("Unexpected value: " + type);
            }
        }
        return builder.build();
    }
}
