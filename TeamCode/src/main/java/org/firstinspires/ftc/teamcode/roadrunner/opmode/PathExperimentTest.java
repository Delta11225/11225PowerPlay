package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class PathExperimentTest extends LinearOpMode {

    public static double DISTANCE = 20; //in

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Vector2d vector = new Vector2d(-30, 20);
        Pose2d startPose= new Pose2d(vector, 0);

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;


            Trajectory traj = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(0,0), Math.toRadians(180))
                    .build();

            drive.followTrajectory(traj);

            sleep(2000);

            Trajectory mid = drive.trajectoryBuilder(traj.end(),Math.toRadians(180))
                    .strafeTo(new Vector2d(30,0))
                    .build();

            drive.followTrajectory(mid);

            sleep(2000);

            drive.followTrajectory(
              drive.trajectoryBuilder(mid.end())
                      .splineTo(vector, Math.toRadians(90))
                      .build()
            );
    }
}