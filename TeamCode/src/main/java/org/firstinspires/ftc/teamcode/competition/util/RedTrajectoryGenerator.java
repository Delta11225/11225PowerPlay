package org.firstinspires.ftc.teamcode.competition.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.competition.types.PathType;
import org.firstinspires.ftc.teamcode.competition.types.ParkingMethod;
import org.firstinspires.ftc.teamcode.competition.types.StartPosition;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;

public class RedTrajectoryGenerator extends TrajectoryGenerator {
    StartPosition position;
    ParkingMethod parkingMethod;

    public RedTrajectoryGenerator(SampleMecanumDrive drive, StartPosition position, ParkingMethod parkingMethod) {
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
            Vector2d vector = new Vector2d(15.0, -61.5);
            Pose2d startPose = new Pose2d(vector, Math.toRadians(90));

            drive.setPoseEstimate(startPose);

            // moving to the shipping hub
            generateTrajectoryListItem(-16.5, -39.5, 90, PathType.LINE_TO_LINEAR, trajectory1);
            if (parkingMethod == ParkingMethod.WALL) {
                // Forward movement to avoid carossel
                //  generateTrajectoryListItem(-50, 56.25, 0, PathType.LINE_TO_LINEAR, trajectory3);
                // getting in position to cross the field and park
                generateTrajectoryListItem(-7, -68, 0, PathType.LINE_TO_LINEAR, trajectory3);
                // hugging the wall and moving into the warehouse
                generateTrajectoryListItem(50, -68.3, 0, PathType.LINE_TO_LINEAR, trajectory3);

            } else if (parkingMethod == ParkingMethod.BARRIER) {
                // getting in position to cross the field and park
                generateTrajectoryListItem(-12, -44, 0, PathType.LINE_TO_LINEAR, trajectory3);
                generateTrajectoryListItem(60, -42, 0, PathType.LINE_TO_LINEAR, trajectory3);
            }

            ArrayList<Trajectory> compTraj1 = compileTrajectoryList(startPose, trajectory1);
            ArrayList<Trajectory> compTraj2 = new ArrayList<>();
            ArrayList<Trajectory> compTraj3 = compileTrajectoryList(compTraj1.get(compTraj1.size() - 1).end(), trajectory3);


            finalTrajs.add(compTraj1);
            finalTrajs.add(compTraj2);
            finalTrajs.add(compTraj3);


        } else if (position == StartPosition.FRONT) {
            Vector2d vector = new Vector2d(-32.5, -61.5);
            Pose2d startPose = new Pose2d(vector, Math.toRadians(90));

            drive.setPoseEstimate(startPose);

            // moving to the shipping hub
            generateTrajectoryListItem(-65,  -50, PathType.LINE_TO_CONSTANT, trajectory1);
//            generateTrajectoryListItem(-58,  -39, 90, PathType.LINE_TO_LINEAR, trajectory1);
            generateTrajectoryListItem(-58,-21, 0, PathType.LINE_TO_LINEAR, trajectory1);

            generateTrajectoryListItem(-33, -21, 0, PathType.LINE_TO_LINEAR, trajectory1);


            // Approaching to duck wheel
            generateTrajectoryListItem(-60, -21, 0, PathType.LINE_TO_LINEAR, trajectory2);
            generateTrajectoryListItem(-63, -55.5, 90, PathType.LINE_TO_LINEAR, trajectory2);
            // At duck wheel
            generateTrajectoryListItem(-65, -57.5, 90, PathType.LINE_TO_LINEAR, trajectory2);

            if (parkingMethod == ParkingMethod.WALL) {
                // Forward movement to avoid carossel
                generateTrajectoryListItem(-40, -52.5, 0, PathType.LINE_TO_LINEAR, trajectory3);
                // getting in position to cross the field and park
                generateTrajectoryListItem(-40, -70, 0, PathType.LINE_TO_LINEAR, trajectory3);
                // hugging the wall and moving into the warehouse
                generateTrajectoryListItem(50, -74, 0, PathType.LINE_TO_LINEAR, trajectory3);
            } else if (parkingMethod == ParkingMethod.BARRIER) {
                // Forward movement to avoid carossel
                generateTrajectoryListItem(-50, -44, 0, PathType.LINE_TO_LINEAR, trajectory3);
                // Forward move to warehouse over barrier
                generateTrajectoryListItem(50, -44, 0, PathType.LINE_TO_LINEAR, trajectory3);
            }  else if (parkingMethod == ParkingMethod.STORAGE) {
                generateTrajectoryListItem(-64, -35, 0, PathType.LINE_TO_LINEAR, trajectory3);
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
