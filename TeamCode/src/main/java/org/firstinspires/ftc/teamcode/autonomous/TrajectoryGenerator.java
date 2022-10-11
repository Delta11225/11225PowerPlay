package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Hardware22;
import org.firstinspires.ftc.teamcode.util.Hardware23;

import kotlin.NotImplementedError;

public class TrajectoryGenerator {
    private final Hardware23 robot;
    private final SampleMecanumDrive drive;
    private AutoState autoState;
    private ParkingPosition parkPos;

    public TrajectoryGenerator(Hardware23 robot, AutoState autoState, ParkingPosition parkPos) {
        this.robot = robot;
        this.drive = robot.drive;
        this.autoState = autoState;
        this.parkPos = parkPos;
    }

    public TrajectorySequence generateTrajectories() {
        TrajectorySequenceBuilder gen = null;
        switch (autoState.color) {
            case RED:
                throw new NotImplementedError();
//            break;
            case BLUE:
                return getBlueTrajectories();
        }
        return null;
    }

    private TrajectorySequence getBlueTrajectories() {
        TrajectorySequenceBuilder gen = null;
        switch (autoState.position) {
            case FRONT:
                Pose2d startPose = new Pose2d(-35, 70-(12.25/2.0), Math.toRadians(270));
                drive.setPoseEstimate(startPose);
                gen = drive.trajectorySequenceBuilder(startPose);
                switch (parkPos) {
                    case ONE:
                        gen.strafeLeft(35-16)
                                .splineToConstantHeading(new Vector2d(-12, 36), Math.toRadians(270));
                        break;
                    case TWO:
                        gen.forward(70-(12.25/2.0) - 36);
                        break;
                    default:
                        gen.strafeRight(35-16)
                                .splineToConstantHeading(new Vector2d(-59, 36), Math.toRadians(270));
                        break;
//                    case HOW_ON_EARTH:
                        // TODO maybe have it do a default so it'll park somewhere instead of crashing
//                        throw new IllegalStateException("??????????????????????");
                }
                break;
            case BACK:
                throw new NotImplementedError();
//                  break;
        }
        return gen.build();
    }
}
