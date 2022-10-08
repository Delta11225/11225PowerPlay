package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Hardware22;

public class TrajectoryGenerator {
   private final Hardware22 robot;
   private final SampleMecanumDrive drive;
   private AutoState autoState;
   private ParkingPosition parkPos;

   public TrajectoryGenerator(Hardware22 robot, AutoState autoState, ParkingPosition parkPos) {
      this.robot = robot;
      this.drive = robot.drive;
      this.autoState = autoState;
      this.parkPos = parkPos;
   }

   public TrajectorySequence generateTrajectories() {
      return null;
   }
}
