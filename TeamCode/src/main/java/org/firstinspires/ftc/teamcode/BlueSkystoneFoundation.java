package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous(name= "Blue Skystone Foundation", group="Sky autonomous")
@Disabled
@Deprecated
public class BlueSkystoneFoundation extends AutoMethodsYellow {
    HunterHardware robot = new HunterHardware();// Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();


    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = .6f/8f;
    private static float rectWidth = 1.5f/8f;

    private static float offsetX = 0.85f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f/8f+offsetX, 4f/8f+offsetY};
    private static float[] rightPos = {6f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera webcam;

    String positionSkystone = "";

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.rearLeft.setPower(0);
        robot.rearRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.collectorLeft.setPower(0);
        robot.collectorRight.setPower(0);

        robot.rightServo.setPosition(0);
        robot.leftServo.setPosition(1);
// Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver"); //
        telemetry.update();
        robot.armServo.setPosition(.75);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        runtime.reset();

            telemetry.addData("Values", valLeft+"   "+valMid+"   "+valRight);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
            telemetry.update();
        if(valLeft == 0 && valMid != 0 && valRight != 0){
            positionSkystone = "left";
            telemetry.addLine("skystone left");
            telemetry.update();
        } else if(valLeft != 0 && valMid == 0 && valRight != 0) {
            positionSkystone = "center";
            telemetry.addLine("skystone center");
            telemetry.update();
        }else if(valLeft != 0 && valMid != 0 && valRight == 0) {
            positionSkystone = "right";
            telemetry.addLine("skystone right");
            telemetry.update();
        }
        sleep(100);

        //initial approach to skystone

        encoderDrive_Straight_Acc(0.7,50,.2,5,.5);
     /*   encoder_ArmRotate(0.5,40, "up",5);
        sleep(100);
        robot.armServo.setPosition(.75);
        sleep(200);
        encoder_ArmRotate(0.5,30, "down",1); */

       // encoderDrive_Straight_Dump(0.3,85,5);


        //code specific to each location skystone
        if(positionSkystone=="right"){
            encoderDrive_Side_Acc(.7,7,.2,5,.25);
            collectorIn();
            encoderDrive_Straight_Acc(0.5,55,.2,5,.25);
            encoderDrive_Straight_Acc(0.4,-43,.2,5,.25);
            collectorOff();
            encoderDrive_Side_Acc(1,-155,.1,5,.20);
            collectorOut();
            encoderTurn_Acc("clockwise",.7,90,.2,5,.5);
            collectorOff();
            encoderDrive_Straight_Acc(1,172,.1,5,.2);
            encoderDrive_Side_Acc(1,-39,.1,5,.2);
            collectorIn();
            encoderDrive_Straight_Acc(0.7,19,.1,5,.2);
            collectorOff();
            encoderDrive_Side_Acc(1,42,.1,5,.2);
            encoderDrive_Straight_Acc(1,-275, .1,7,.25);
            encoderTurn_Acc("clockwise",.7,89,.2,5,.35);
            encoderDrive_Straight(.2,-23, 5);
            collectorOut();
            BlueFoundationEnding();
            collectorOff();

        } else if(positionSkystone=="center"){
            encoderDrive_Side(.5,-8,5);
            collectorIn();
            encoderDrive_Straight_Acc(0.5,47,.2,5,.25);
            encoderDrive_Straight_Acc(0.5,-42,.2,5,.35);
            collectorOff();
            encoderDrive_Side_Acc(1,-132,.1,5,.20);
            collectorOut();
            encoderDrive_Straight(.6,5,5);
            //encoderDrive_Straight(.75,-2,5);
            //collectorOff();
            ///Just dumped first stone
            encoderDrive_Side_Acc(1,190,.1,5,.20);
            collectorIn();
            encoderDrive_Straight_Acc(0.5,41,.2,5,.25);
            encoderDrive_Straight_Acc(0.5,-39,.2,5,.25);
            collectorOff();
            encoderTurn_Acc("clockwise",.7,91.5,.2,5,.5);
            encoderDrive_Straight_Acc(1,-260, .1,7,.1);//added
            encoderTurn_Acc("clockwise",.7,90,.1,5,.35);
            encoderDrive_Straight(.2,-29, 5);
            collectorOut();
            BlueFoundationEnding();
            collectorOff();

        } else if(positionSkystone == "left"){
            encoderDrive_Side(.7,-30,5);
            collectorIn();
            encoderDrive_Straight_Acc(0.4,51,.2,5,.25);
            encoderDrive_Straight_Acc(0.4,-43,.2,5,.35);
            collectorOff();
            encoderDrive_Side_Acc(.8,-120,.3,5,.3);
            collectorOut();
            encoderDrive_Straight(.4,5,5);
            ///Just dumped first stone
            encoderDrive_Side_Acc(1,176,.1,5,.20);
            collectorIn();
            encoderDrive_Straight_Acc(0.5,35,.2,5,.25);
            encoderDrive_Straight_Acc(0.5,-39,.2,5,.35);
            collectorOff();
            //encoderTurn_Acc("clockwise",.7,91.5,.2,5,.5);
            encoderDrive_Side_Acc(1,-265, .2,7,.25);//added
            encoderTurn_Acc("clockwise",.6,180,.1,5,.35);
            encoderDrive_Straight(.2,-27, 5);
            collectorOut();
            BlueFoundationEnding();
            collectorOff();
        }

    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
            valMid = (int)pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

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
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(rightPos[0]-rectWidth/2),
                            input.rows()*(rightPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(rightPos[0]+rectWidth/2),
                            input.rows()*(rightPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}
