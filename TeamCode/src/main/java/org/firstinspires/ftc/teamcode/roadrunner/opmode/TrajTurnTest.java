package org.firstinspires.ftc.teamcode.roadrunner.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")
public class TrajTurnTest extends LinearOpMode {

    public static double DISTANCE = 20; //in

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Vector2d vector = new Vector2d(-38.25, 62);
        Pose2d startPose= new Pose2d(vector, Math.toRadians(270));

        //drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                //.forward(10)
                //.turn(Math.toRadians(90))
                //.splineToConstantHeading(new Vector2d(-10,40), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-47,47, Math.toRadians(90)), Math.toRadians(90))
                .build();
        drive.followTrajectorySequence(traj);

    }
}