package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.ParkingPosition;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

@Disabled
@Autonomous
public class WebcamTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    OpenCvCamera webcam;

    @Override
    public void runOpMode() {

//        telemetry = new TelemWrapper(telemetry, FtcDashboard.getInstance().getTelemetry());
        // servoTest = hardwareMap.get(Servo.class, "servoTest");
        // servoTest.setPosition(0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        DetectionPipeline pipeline = new DetectionPipeline();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.setPipeline(pipeline);
                //the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 0);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();
        runtime.reset();

        telemetry.addData("Raw vals (R, G, B)", Arrays.toString(pipeline.getRawVals()));
        telemetry.addData("Result", pipeline.getLastPos());

        telemetry.update();
        sleep(1000);
    }

    private static class DetectionPipeline extends OpenCvPipeline {
        private static final float[] midPos = {1f/2f, 1f/2f};

        private static float rectHeight = 1f/8f;
        private static float rectWidth =  1f/8f;

        private int channelIndexLast = -1;
        private Double[] rawVals = new Double[] {0d, 0d, 0d};
        private ParkingPosition posLast = ParkingPosition.HOW_ON_EARTH;

        Mat displayMat = new Mat();
        Mat rMat = new Mat(); // Channel 0
        Mat gMat = new Mat(); // Channel 1
        Mat bMat = new Mat(); // Channel 2

        Mat hsvMat = new Mat();
        Mat hMat = new Mat(); // Channel 0
//            Mat sMat = new Mat(); // Channel 1
//            Mat vMat = new Mat(); // Channel 2

        Mat satMat = new Mat();

        @Override
        public Mat processFrame(Mat input) {
            // Do HSV stuff, crank S and V
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(hsvMat, hMat, 0);
            // We don't need these channels
//            Core.extractChannel(hsvMat, sMat, 1);
//            Core.extractChannel(hsvMat, vMat, 2);

            // Get mat of all ones of the same size as input to set saturation and value (brightness)
            // to max
            Mat ones = new Mat(input.rows(), input.cols(), hMat.type(), new Scalar(255));

            // Put the mats in the proper order as merge() expects a list
            List<Mat> matList = Arrays.asList(
                    hMat,
                    ones,
                    ones
            );

            Core.merge(matList, satMat);
//            satMat.copyTo(all);

            // Convert back to RGB for display reasons
            Imgproc.cvtColor(satMat, displayMat, Imgproc.COLOR_HSV2RGB);
            Core.extractChannel(displayMat, rMat, 0);
            Core.extractChannel(displayMat, gMat, 1);
            Core.extractChannel(displayMat, bMat, 2);

            // Create middle point
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));

            // Draw circle on specified point
            Imgproc.circle(displayMat, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            // Draw rectangle around point
            Imgproc.rectangle(
                    displayMat,
                    new Point(
                            input.cols()*(midPos[0]- rectWidth /2),
                            input.rows()*(midPos[1]- rectHeight /2)),
                    new Point(
                            input.cols()*(midPos[0]+ rectWidth /2),
                            input.rows()*(midPos[1]+ rectHeight /2)),
                    new Scalar(0, 255, 0), 3);

            // Get proper channel index
            // There are probably easier ways to do this with hue distance or something, but
            // I am too lazy to even try to implement those
            Double[] vals = new Double[] {
                    rMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]))[0],
                    gMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]))[0],
                    bMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]))[0]
            };

            int maxAt = 0;

            for (int i = 0; i < vals.length; i++) {
                maxAt = vals[i] > vals[maxAt] ? i : maxAt;
            }

            channelIndexLast = maxAt;

            // A mapping between channel index (r,g,b) and position (1,2,3). Change it if mapping
            // changes.
            HashMap<Integer, ParkingPosition> posMappings = new HashMap<>();
            posMappings.put(0, ParkingPosition.THREE);
            posMappings.put(1, ParkingPosition.ONE);
            posMappings.put(2, ParkingPosition.TWO);

            posLast = posMappings.getOrDefault(maxAt, ParkingPosition.HOW_ON_EARTH);

            // Set these for debugging
            rawVals = vals;
            return displayMat;
            // return input;
        }

        public int getLastChannelMax() {
            return channelIndexLast;
        }

        public Double[] getRawVals() {
            return rawVals;
        }

        public ParkingPosition getLastPos() {
            return posLast;
        }
    }
}
