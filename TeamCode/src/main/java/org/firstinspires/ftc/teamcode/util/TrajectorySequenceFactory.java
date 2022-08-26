package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.PathContinuityViolationException;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;

import kotlin.NotImplementedError;

class TrajectorySequenceFactory {
   private static Double endTangentDefault = 0d;

   private final ArrayList<Double[]> internalPathRepresentation;
   private final SampleMecanumDrive drive;
   private final Telemetry telemetry;
   private final Pose2d startPose;

   public TrajectorySequenceFactory(SampleMecanumDrive drive, Telemetry telemetry, Pose2d startPose) {
      // The Double arrays here have the following structure:
      //  [x, y, heading, pathType, endTangent]
       this.internalPathRepresentation= new ArrayList<>();
       this.drive = drive;
       this.telemetry = telemetry;
       this.startPose = startPose;

       // If first navigation is a delta instead of a pose, we need this for reference
       internalPathRepresentation.add(new Double[]{startPose.getX(), startPose.getY(), startPose.getHeading(), -1d});
   }

   public TrajectorySequenceFactory navigateTo(Pose2d pose) {
      return this.navigateTo(pose, PathType.SPLINE_TO_SPLINE, endTangentDefault);
   }

   public TrajectorySequenceFactory navigateTo(Vector2d delta) {
      return this.navigateTo(delta, PathType.SPLINE_TO_SPLINE);
   }

   public TrajectorySequenceFactory navigateTo(Pose2d pose, PathType pathType, Double endTangent) {
      internalPathRepresentation.add(
              new Double[]{pose.getX(), pose.getY(), pose.getHeading(), (double) pathType.getValue(), endTangent}
      );
      return this;
   }

   public TrajectorySequenceFactory navigateTo(Vector2d delta, PathType pathType) {
      return this.navigateTo(new Pose2d(delta.getX(), delta.getY(), getLatestPose().getHeading()), pathType, endTangentDefault);
   }

   private Pose2d getLatestPose() {
      Double[] lastArr = internalPathRepresentation.get(internalPathRepresentation.size() - 1);
      return composePoseFromArr(lastArr);
   }

   public TrajectorySequence build() {
      TrajectorySequenceBuilder builder;

      // Test for continuity
      try {
         TrajectoryBuilder builderTest = null;
         Trajectory traj = null;

         for (Double[] posArr : internalPathRepresentation) {
            Pose2d currPose = composePoseFromArr(posArr);
            switch ((int) Math.round(posArr[3])) {
               case -1:
                  builderTest = drive.trajectoryBuilder(currPose);
                  break;
               case 0:
                  builderTest.lineToLinearHeading(currPose);
                  break;
               case 1:
                  builderTest.lineToSplineHeading(currPose);
                  break;
               case 2:
                  builderTest.splineToLinearHeading(currPose, posArr[4]);
                  break;
               case 3:
                  builderTest.splineToSplineHeading(currPose, posArr[4]);
                  break;
               default:
                  throw new IllegalStateException("Unexpected path type: " + (int) Math.round(posArr[3]));
            }
            traj = builderTest.build();
         }
      } catch (PathContinuityViolationException e) {
         throw new NotImplementedError();
//         throw e;
      }
      return null;
   }

   private Pose2d composePoseFromArr(Double[] posArr) {
      return new Pose2d(posArr[0], posArr[1], posArr[2]);
   }
}
