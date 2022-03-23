package org.firstinspires.ftc.teamcode.autonomous.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware22;
import org.firstinspires.ftc.teamcode.autonomous.competition.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.autonomous.enums.PathType;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")
public class AutoBlueTestWithObjectDetect extends LinearOpMode {
    Hardware22 robot;
    SampleMecanumDrive drive;
    TrajectoryGenerator generator;
    OpenCvCamera webcam;

    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
//    private static int valRight = -1;

    private static float rectHeight = 1f/8f;
    private static float rectWidth =  1f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {7f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2.75f/8f+offsetX, 4f/8f+offsetY};
    //  private static float[] rightPos = {7f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware22(hardwareMap);
        drive = robot.drive;
        generator = robot.generator;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();
        webcam.setPipeline(new SamplePipeline());

        //the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);


        /*
        //code needed for camera to display on FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(webcam, 10);
        telemetry.update();
       */

        telemetry.addData("Values", valLeft+"   "+valMid);
        telemetry.update();
        Vector2d vector = new Vector2d(-32.25, 61.5);
        Pose2d startPose = new Pose2d(vector, Math.toRadians(270));

        drive.setPoseEstimate(startPose);
        ArrayList<Double[]> trajectory1 = new ArrayList<>();
        generator.generateTrajectoryListItem(-30, 47, 310, 0, PathType.SPLINE_TO_LINEAR, trajectory1);
        generator.generateTrajectoryListItem(-22, 36.5, PathType.LINE_TO_CONSTANT, trajectory1);
        // Approaching to duck wheel
        generator.generateTrajectoryListItem(-60, 56.25, 0, PathType.LINE_TO_LINEAR, trajectory1);
        // At duck wheel
        generator.generateTrajectoryListItem(-60, 56.25 + 1.7, 0, PathType.LINE_TO_LINEAR, trajectory1);


        ArrayList<Double[]> trajectory2 = new ArrayList<>();
        generator.generateTrajectoryListItem(-50, 66.7, 0, PathType.LINE_TO_LINEAR, trajectory2);
        generator.generateTrajectoryListItem(50, 67.2, 0, PathType.LINE_TO_LINEAR, trajectory2);

        ArrayList<Trajectory> compiled1 = generator.compileTrajectoryList(startPose, trajectory1);
        ArrayList<Trajectory> compiled2 = generator.compileTrajectoryList(compiled1.get(compiled1.size() - 1).end(), trajectory2);

        waitForStart();

        //duck detection

        if (valLeft == 255) {
            telemetry.addData("Position", "Left");
            telemetry.update();
            sleep(1000);
        }
        else if (valMid == 255) {
            telemetry.addData("Position", "Middle");
            telemetry.update();
            sleep(1000);
        }

        else {
            telemetry.addData("Position", "Right");
            telemetry.update();
            sleep(1000);
        }

        telemetry.update();

//        if (isStopRequested()) return;
        generator.executeTrajectoryList(compiled1);

        sleep(500);
        robot.towerMotor.setPower(Constants.towerWheelSpeedEndgame);
        sleep(2000);
        robot.towerMotor.setPower(0);
        sleep(500);

        generator.executeTrajectoryList(compiled2);
    }
    class SamplePipeline extends OpenCvPipeline {
        Mat yCbCr = new Mat();
        Mat yMat = new Mat();
        Mat CbMat = new Mat();
        Mat CrMat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.cvtColor(input, yCbCr, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCr, yMat, 0);//extracts cb channel as black and white RGB
            Core.extractChannel(yCbCr, CrMat, 1);//extracts cb channel as black and white RGB
            Core.extractChannel(yCbCr, CbMat, 2);//extracts cb channel as black and white RGB
            Imgproc.threshold(CbMat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);
            //any pixel with a hue value less than 102 is being set to 0 (yellow)
            //any pixel with a hue value greater than 102 is being set to 255(blue)
            //Then swaps the blue and the yellows with the binary inv line
            CbMat.copyTo(all);//copies mat object

            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            //   double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            //   valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            //   Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            // Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(midPos[0]-rectWidth/2),
                            input.rows()*(midPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(midPos[0]+rectWidth/2),
                            input.rows()*(midPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
         /*   Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3); */


            //return input; // this is the line that declares which image is returned to the viewport (DS)
            //return CbMat;
            return all;
        }
    }
}