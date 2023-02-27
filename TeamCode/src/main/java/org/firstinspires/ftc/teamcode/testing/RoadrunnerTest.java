package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Hardware23;

@Autonomous
@Disabled
public class RoadrunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware23 robot = new Hardware23(hardwareMap);

        waitForStart();

        TrajectorySequenceBuilder trajGen = robot.drive.trajectorySequenceBuilder(new Pose2d(0,0,0));
        trajGen.forward(50);
        trajGen.back(50);
        TrajectorySequence trajs = trajGen.build();

        robot.drive.followTrajectorySequence(trajs);
    }
}
