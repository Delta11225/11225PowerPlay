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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

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
      //  [x, y, heading, pathType, endTangent (will be -999 if not applicable)]
       this.internalPathRepresentation= new ArrayList<>();
       this.drive = drive;
       this.telemetry = telemetry;
       this.startPose = startPose;

       // If first navigation is a delta instead of a pose, we need this as a starting reference
       internalPathRepresentation.add(new Double[]{startPose.getX(), startPose.getY(), startPose.getHeading(), -999d});
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
      return this.navigateTo(
              new Pose2d(delta.getX(),
              delta.getY(),
              getLatestPose().getHeading()),
              pathType,
              // If the pathType is a spline movement type (NOT spline heading), then we need to
              // pass the default end tangent into the method, otherwise we need to put -999 as that
              // signifies that there is no endTangent. Technically, this is not strictly required,
              // but it helps with readability.
              pathType == PathType.SPLINE_TO_SPLINE || pathType == PathType.SPLINE_TO_LINEAR ? endTangentDefault : -999d
      );
   }

   private Pose2d getLatestPose() {
      Double[] lastArr = internalPathRepresentation.get(internalPathRepresentation.size() - 1);
      return composePoseFromArr(lastArr);
   }

   public TrajectorySequence build() {
      TrajectorySequenceBuilder builder;
      TrajectorySequence sequence = null;
      Trajectory traj = null;

      // Test for path continuity
      try {
         TrajectoryBuilder builderTest = null;

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
         telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
         telemetry.addLine("<p style='color:yellow;'>TEST</p>");
         telemetry.update();
         traj = null;
//         throw e;
      }
      if (traj == null) {
         throw new NotImplementedError();
      } else {
         sequence = drive.trajectorySequenceBuilder(composePoseFromArr(internalPathRepresentation.get(0)))
                 .addTrajectory(traj)
                 .build();
      }
      return sequence;
   }

   private Pose2d composePoseFromArr(Double[] posArr) {
      return new Pose2d(posArr[0], posArr[1], posArr[2]);
   }

   public static void setDefaultEndTangent(double endTangentDefault) {
      TrajectorySequenceFactory.endTangentDefault = endTangentDefault;
   }
}
