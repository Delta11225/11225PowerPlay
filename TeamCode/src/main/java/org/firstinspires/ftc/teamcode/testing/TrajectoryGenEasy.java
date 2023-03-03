package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.types.AutoState;
import org.firstinspires.ftc.teamcode.util.types.ParkingPosition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Hardware23;

public class TrajectoryGenEasy {
    Hardware23 robot;
    Telemetry telemetry;

    public TrajectoryGenEasy(Hardware23 robot, Telemetry telemetry) {
        this.robot = robot;
        this.telemetry = telemetry;
    }

    public TrajectorySequence getAppropriateTrajectory(AutoState autoState, ParkingPosition parkPos) {
        TrajectorySequenceBuilder trajGen = robot.drive.trajectorySequenceBuilder(new Pose2d(0,0,0));
        trajGen.forward(50);
        trajGen.back(50);
        TrajectorySequence trajs = trajGen.build();
        robot.drive.setPoseEstimate(trajs.start());
        return trajs;
    }
}
