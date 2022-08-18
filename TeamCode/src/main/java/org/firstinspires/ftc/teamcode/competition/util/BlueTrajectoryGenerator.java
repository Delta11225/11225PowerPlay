package org.firstinspires.ftc.teamcode.competition.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.competition.types.PathType;
import org.firstinspires.ftc.teamcode.competition.types.ParkingMethod;
import org.firstinspires.ftc.teamcode.competition.types.StartPosition;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;

public class BlueTrajectoryGenerator extends TrajectoryGenerator {
    StartPosition position;
    ParkingMethod parkingMethod;

    public BlueTrajectoryGenerator(SampleMecanumDrive drive, StartPosition position, ParkingMethod parkingMethod) {
        super(drive);
        this.position = position;
        this.parkingMethod = parkingMethod;
    }

    public ArrayList<ArrayList<Trajectory>> generateTrajectories() {
        ArrayList<Double[]> trajectory1 = new ArrayList<>();
        ArrayList<Double[]> trajectory2 = new ArrayList<>();
        ArrayList<Double[]> trajectory3 = new ArrayList<>();

        ArrayList<ArrayList<Trajectory>> finalTrajs = new ArrayList<>();

        if (position == StartPosition.BACK) {
            Vector2d vector = new Vector2d(8.0, 61.5);
            Pose2d startPose = new Pose2d(vector, Math.toRadians(270));

            drive.setPoseEstimate(startPose);

            // moving to the shipping hub
            generateTrajectoryListItem(-14, 39.5, 270, PathType.LINE_TO_LINEAR, trajectory1);
            if (parkingMethod == ParkingMethod.WALL) {
                // Forward movement to avoid carossel
                //  generateTrajectoryListItem(-50, 56.25, 0, PathType.LINE_TO_LINEAR, trajectory3);
                // getting in position to cross the field and park
                generateTrajectoryListItem(-7, 68, 0, PathType.LINE_TO_LINEAR, trajectory3);
                // hugging the wall and moving into the warehouse
                generateTrajectoryListItem(50, 68.3, 0, PathType.LINE_TO_LINEAR, trajectory3);

            } else if (parkingMethod == ParkingMethod.BARRIER) {
                // getting in position to cross the field and park
                generateTrajectoryListItem(-12, 46, 0, PathType.LINE_TO_LINEAR, trajectory3);
                generateTrajectoryListItem(60, 46, 0, PathType.LINE_TO_LINEAR, trajectory3);
            }

            ArrayList<Trajectory> compTraj1 = compileTrajectoryList(startPose, trajectory1);
            ArrayList<Trajectory> compTraj2 = new ArrayList<>();
            ArrayList<Trajectory> compTraj3 = compileTrajectoryList(compTraj1.get(compTraj1.size() - 1).end(), trajectory3);


            finalTrajs.add(compTraj1);
            finalTrajs.add(compTraj2);
            finalTrajs.add(compTraj3);


        } else if (position == StartPosition.FRONT) {
            Vector2d vector = new Vector2d(-32.25, 61.5);
            Pose2d startPose = new Pose2d(vector, Math.toRadians(270));

            drive.setPoseEstimate(startPose);

            // moving to the shipping hub
            generateTrajectoryListItem(-57, 38, 270, PathType.LINE_TO_LINEAR, trajectory1);
            generateTrajectoryListItem(-57, 22, 0, PathType.LINE_TO_LINEAR, trajectory1);

            generateTrajectoryListItem(-25, 22, 0, PathType.LINE_TO_LINEAR, trajectory1);


            // Approaching to duck wheel
            generateTrajectoryListItem(-57, 23, 0, PathType.LINE_TO_LINEAR, trajectory2);
            generateTrajectoryListItem(-60, 62, 0, PathType.LINE_TO_LINEAR, trajectory2);
            // At duck wheel
            generateTrajectoryListItem(-60, 62 + 1.5, 0, PathType.LINE_TO_LINEAR, trajectory2);

            if (parkingMethod == ParkingMethod.WALL) {
                // Forward movement to avoid carossel
                generateTrajectoryListItem(-30, 60, 0, PathType.LINE_TO_LINEAR, trajectory3);
                // getting in position to cross the field and park
                generateTrajectoryListItem(-30, 72, 0, PathType.LINE_TO_LINEAR, trajectory3);
                // hugging the wall and moving into the warehouse
                generateTrajectoryListItem(50, 76, 0, PathType.LINE_TO_LINEAR, trajectory3);
            } else if (parkingMethod == ParkingMethod.BARRIER) {
                // Forward movement to avoid carossel
                generateTrajectoryListItem(-50, 47, 0, PathType.LINE_TO_LINEAR, trajectory3);
                // Forward move to warehouse over barrier
                generateTrajectoryListItem(50, 47, 0, PathType.LINE_TO_LINEAR, trajectory3);
            } else if (parkingMethod == ParkingMethod.STORAGE) {
                generateTrajectoryListItem(-58, 38, 0, PathType.LINE_TO_LINEAR, trajectory3);
            }

            ArrayList<Trajectory> compTraj1 = compileTrajectoryList(startPose, trajectory1);
            ArrayList<Trajectory> compTraj2 = compileTrajectoryList(compTraj1.get(compTraj1.size() - 1).end(), trajectory2);
            ArrayList<Trajectory> compTraj3 = compileTrajectoryList(compTraj2.get(compTraj2.size() - 1).end(), trajectory3);

            finalTrajs.add(compTraj1);
            finalTrajs.add(compTraj2);
            finalTrajs.add(compTraj3);
        }
        return finalTrajs;
    }
}