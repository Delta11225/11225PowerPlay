package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Hardware22;
import org.firstinspires.ftc.teamcode.util.Hardware23;

// This class just runs the setWeightedDrivePower method with different values to attempt to
// decipher how it works
@Autonomous
public class WeightedDrivePowerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware23 robot = new Hardware23(hardwareMap);
        telemetry.addLine("ready");
        telemetry.update();
        waitForStart();

        telemetry.addLine("1, 0, 0 degs");
        telemetry.update();
        robot.drive.setWeightedDrivePower(new Pose2d(1, 0, 0));
        sleep(1000);
        robot.drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        sleep(1000);

        telemetry.addLine("0, 1, 0 degs");
        telemetry.update();
        robot.drive.setWeightedDrivePower(new Pose2d(1, 0, 0));
        sleep(1000);
        robot.drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
        sleep(1000);


        telemetry.addLine("1, 0, 90 degs");
        telemetry.update();
        robot.drive.setWeightedDrivePower(new Pose2d(1, 0, Math.toRadians(90)));
        sleep(1000);
        robot.drive.setWeightedDrivePower(new Pose2d(0, 0, Math.toRadians(90)));
        sleep(1000);

        telemetry.addLine("0, 1, 90 degs");
        telemetry.update();
        robot.drive.setWeightedDrivePower(new Pose2d(1, 0, Math.toRadians(90)));
        sleep(1000);
        robot.drive.setWeightedDrivePower(new Pose2d(0, 0, Math.toRadians(90)));
        sleep(1000);


        telemetry.addLine("1, 0, 180 degs");
        telemetry.update();
        robot.drive.setWeightedDrivePower(new Pose2d(1, 0, Math.toRadians(180)));
        sleep(1000);
        robot.drive.setWeightedDrivePower(new Pose2d(0, 0, Math.toRadians(180)));
        sleep(1000);

        telemetry.addLine("0, 1, 180 degs");
        telemetry.update();
        robot.drive.setWeightedDrivePower(new Pose2d(1, 0, Math.toRadians(180)));
        sleep(1000);
        robot.drive.setWeightedDrivePower(new Pose2d(0, 0, Math.toRadians(180)));
        sleep(1000);


        telemetry.addLine("1, 0, 270 degs");
        telemetry.update();
        robot.drive.setWeightedDrivePower(new Pose2d(1, 0, Math.toRadians(270)));
        sleep(1000);
        robot.drive.setWeightedDrivePower(new Pose2d(0, 0, Math.toRadians(270)));
        sleep(1000);

        telemetry.addLine("0, 1, 270 degs");
        telemetry.update();
        robot.drive.setWeightedDrivePower(new Pose2d(1, 0, Math.toRadians(270)));
        sleep(1000);
        robot.drive.setWeightedDrivePower(new Pose2d(0, 0, Math.toRadians(270)));
        sleep(1000);
    }
}
