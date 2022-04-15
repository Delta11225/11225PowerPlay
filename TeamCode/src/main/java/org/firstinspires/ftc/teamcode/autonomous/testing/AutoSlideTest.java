package org.firstinspires.ftc.teamcode.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.competition.util.Hardware22;
import org.firstinspires.ftc.teamcode.competition.util.TrajectoryGenerator;
import org.firstinspires.ftc.teamcode.competition.types.Color;
import org.firstinspires.ftc.teamcode.competition.types.ElementPosition;
import org.firstinspires.ftc.teamcode.competition.types.ParkingMethod;
import org.firstinspires.ftc.teamcode.competition.types.StartPosition;
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
@Disabled
@Autonomous
public class AutoSlideTest extends LinearOpMode {

    Color color;
    StartPosition position;
    ParkingMethod parkingMethod;
    ElementPosition elementPosition;
    private Hardware22 robot;
    private int liftEncoderStart;
    long delay = 0;
    //Telemetry telemetry;
    OpenCvCamera webcam;

    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    //private static int valLeft = -1;
    private static int valRight = -1;

    private static float rectHeight = 1f/8f;
    private static float rectWidth =  1f/8f;

    private static float offsetX = 0f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] rightPos = {7f/8f+offsetX, 4f/8f+offsetY};//0 = col, 1 = row
    private static float[] midPos = {3f/8f+offsetX, 4f/8f+offsetY};
    //  private static float[] rightPos = {7f/8f+offsetX, 4f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Hardware22(hardwareMap);
        SampleMecanumDrive drive = robot.drive;
        TrajectoryGenerator generator;
        //set start position for linear slide encoder
        liftEncoderStart = robot.liftMotor.getCurrentPosition();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();
        webcam.setPipeline(new AutoSlideTest.SamplePipeline());

        //the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);


           /*
        //code needed for camera to display on FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(webcam, 10);
          telemetry.update();
        */




        waitForStart();

        determineDuckPosition();
        sleep(500);

        dumpPreloaded();
    }

    private void determineDuckPosition() {
        //duck detection
        if (valRight == 255) {
            telemetry.addData("Position", "Right");
            telemetry.update();
            elementPosition = ElementPosition.RIGHT;
            sleep(200);
        } else if (valMid == 255) {
            telemetry.addData("Position", "Middle");
            telemetry.update();
            elementPosition = ElementPosition.MIDDLE;
            sleep(200);
        } else {
            telemetry.addData("Position", "Left");
            telemetry.update();
            elementPosition = ElementPosition.LEFT;
            sleep(200);
        }
    }

    private void dumpPreloaded() {
        // Encoder Counts, Bottom: 1100
        // Middle 2120
        // Top 3470
        int encTarget;
        if (elementPosition == ElementPosition.LEFT) {
            encTarget = 1100 ;
            //encTarget = 1100 + (int)(liftEncoderStart);

            telemetry.addLine("Dump bottom");
            telemetry.addData("position",robot.liftMotor.getCurrentPosition());
            telemetry.update();
            sleep(2000);

        } else if (elementPosition == ElementPosition.RIGHT) {
            encTarget = 3470;
            //encTarget = 3470 + (int)(liftEncoderStart);
            telemetry.addLine("Dump top");
            telemetry.addData("position",robot.liftMotor.getCurrentPosition());
            telemetry.update();
            sleep(2000);

        } else {
            encTarget = 2120;
            //encTarget = 2120 + (int)(liftEncoderStart);
            telemetry.addLine("Dump middle");
            telemetry.addData("position",robot.liftMotor.getCurrentPosition());
            telemetry.update();
            sleep(2000);

        }
        telemetry.update();

        int encoderUpTarget;

        encoderUpTarget = robot.liftMotor.getCurrentPosition()+ encTarget;

        robot.liftMotor.setTargetPosition(encoderUpTarget);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(.5);
        while (opModeIsActive() && robot.liftMotor.isBusy()) {
        }

        robot.liftMotor.setPower(0);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(2000);
        int encoderDownTarget;

        encoderDownTarget = robot.liftMotor.getCurrentPosition()-encTarget;
        robot.liftMotor.setTargetPosition(encoderDownTarget);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setPower(.5);
        while (opModeIsActive() && robot.liftMotor.isBusy()) {
        }

        robot.liftMotor.setPower(0);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

            //double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[1]), (int)(input.cols()* leftPos[0]));//gets value at circle
            //valLeft = (int)pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[1]), (int)(input.cols()* rightPos[0]));//gets value at circle
            valRight = (int)pixRight[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            //Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            //Imgproc.circle(all, pointLeft,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            /*
            Imgproc.rectangle(//1-3

                    all,
                    new Point(
                            input.cols()*(leftPos[0]-rectWidth/2),
                            input.rows()*(leftPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(leftPos[0]+rectWidth/2),
                            input.rows()*(leftPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
                    */
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


            //return input; // this is the line that declares which image is returned to the viewport (DS)
            //return CbMat;
            return all;
        }
    }
}