package org.firstinspires.ftc.teamcode.roadrunner.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware22;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.testing.teleop.ControlConfig;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
//@Disabled
@Autonomous(group = "drive")
public class TrajTests extends LinearOpMode {

    public static double DISTANCE = 20; //in
    // FIXME make this much MUCH better
    Hardware22 robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware22();
        robot.init(hardwareMap);
        SampleMecanumDrive drive = robot.drive;

        Vector2d vector = new Vector2d(-40.75, 61.5);
        Pose2d startPose = new Pose2d(vector, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;


        Trajectory traj = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-30, 47, Math.toRadians(310)), Math.toRadians(0))
                .build();

        drive.followTrajectory(traj);

        Trajectory traj1 = drive.trajectoryBuilder(traj.end()).lineToConstantHeading(new Vector2d(-22, 36.5))
                .build();
        // At shipping hub
        drive.followTrajectory(traj1);
//        sleep(3000);


        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
                .lineToLinearHeading(new Pose2d(-60, 56.25, Math.toRadians(0)))
                .build();
        // Approaching to duck wheel
        drive.followTrajectory(traj2);

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), true)
                .lineToLinearHeading(new Pose2d(-60, 56.25 + 1.7, Math.toRadians(0)))
                .build();
        // At duck wheel
        drive.followTrajectory(traj3);

        sleep(500);
        robot.towerMotor.setPower(.6);
        sleep(3000);
        robot.towerMotor.setPower(0);
        sleep(500);

        traj3 = drive.trajectoryBuilder(traj2.end(), true)
                .lineToLinearHeading(new Pose2d(-50, 62.7, Math.toRadians(0)))
                .build();
        // Away from duck wheel
        drive.followTrajectory(traj3);

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), true)
                .lineToLinearHeading(new Pose2d(50, 62.7, Math.toRadians(0)))
                .build();
        drive.followTrajectory(traj4);

        sleep(2000);
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .turn(Math.toRadians(90))
                .build();
        drive.followTrajectorySequence(traj5);

    }
}