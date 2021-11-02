package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class TrajTests extends LinearOpMode {

    public static double DISTANCE = 20; //in

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Vector2d vector = new Vector2d(-38.25, 62);
        Pose2d startPose = new Pose2d(vector, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;


        Trajectory traj = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-30, 47, Math.toRadians(305)), Math.toRadians(0))
                .build();

        drive.followTrajectory(traj);
        Trajectory traj1 = drive.trajectoryBuilder(traj.end()).lineToConstantHeading(new Vector2d(-24, 38))
                .build();

        drive.followTrajectory(traj1);


        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
                .lineToLinearHeading(new Pose2d(-60, 55, Math.toRadians(0)))
                .build();
        drive.followTrajectory(traj2);

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), true)
                .lineToLinearHeading(new Pose2d(-50, 61, Math.toRadians(0)))
                .build();
        drive.followTrajectory(traj3);

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), true)
                .lineToLinearHeading(new Pose2d(50, 61, Math.toRadians(0)))
                .build();
        drive.followTrajectory(traj4);

        sleep(2000);
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .turn(Math.toRadians(90))
                .build();
        drive.followTrajectorySequence(traj5);

    }
}