package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp
public class WebcamTest extends OpMode {
    OpenCvCamera camera;
    DetectionPipeline pipeline = new DetectionPipeline();
    private long counter = 0;

    @Override
    public void init() {
        telemetry = FtcDashboard.getInstance().getTelemetry();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1"); // TODO check this
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 0);
                camera.setPipeline(pipeline);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error code", errorCode);
                telemetry.update();
            }
        });
    }

    @Override
    public void loop() {
        try {
            telemetry.addData("Largest channel", pipeline.getLastResult());
            telemetry.addData("Count", counter);
            counter++;
        } catch (Exception e) {
            telemetry.addData("Exception", e.getCause());
            telemetry.update();
            try {
                Thread.sleep(10000);
            } catch (InterruptedException ex) {
                ex.printStackTrace();
            }
        }
//        try {
//            Thread.sleep(50);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        Double[] vals = pipeline.getHsvVals();
//        telemetry.addData("Hue", vals[0]);
//        telemetry.addData("Sat", vals[1]);
//        telemetry.addData("Val", vals[2]);
    }

    static class DetectionPipeline extends OpenCvPipeline {
        private static final float[] midPos = {1f/2f, 1f/2f};

        private int channelIndexLast = -1;
//        private Double[] hsvVals;

        @Override
        public Mat processFrame(Mat input) {
            Mat bgrMat = new Mat();
            Mat bMat = new Mat(); // Channel 0
            Mat gMat = new Mat(); // Channel 1
            Mat rMat = new Mat(); // Channel 2

            Mat hsvMat = new Mat();
            Mat hMat = new Mat(); // Channel 0
//            Mat sMat = new Mat(); // Channel 1
//            Mat vMat = new Mat(); // Channel 2

            Mat all = new Mat();


//            Imgproc.cvtColor(input, bgrMat, Imgproc.COLOR_RGB2BGR);
            Core.extractChannel(input, rMat, 0);
            Core.extractChannel(input, gMat, 1);
            Core.extractChannel(input, bMat, 2);

            // Do HSV stuff, crank S and V
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(hsvMat, hMat, 0);
//            Core.extractChannel(hsvMat, sMat, 1);
//            Core.extractChannel(hsvMat, vMat, 2);
            Mat ones = new Mat(input.rows(), input.cols(), hMat.type(), new Scalar(255));

            List<Mat> matList = Arrays.asList(
                    hMat,
                    ones,
                    ones
            );

            Core.merge(matList, all);

//            rMat.copyTo(all);//copies mat object

            //get values from frame
//            double[] pixMid = rMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]));//gets value at circle
//            valMid = (int)pixMid[0];

            //create three points
            Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            float rectHeight = 1f / 8f;
            float rectWidth = 1f / 8f;
            Imgproc.rectangle(
                    all,
                    new Point(
                            input.cols()*(midPos[0]- rectWidth /2),
                            input.rows()*(midPos[1]- rectHeight /2)),
                    new Point(
                            input.cols()*(midPos[0]+ rectWidth /2),
                            input.rows()*(midPos[1]+ rectHeight /2)),
                    new Scalar(0, 255, 0), 3);

            // Get proper channel index
            Double[] vals = new Double[] {
                    bMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]))[0],
                    gMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]))[0],
                    rMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]))[0]
            };

            int maxAt = 0;

            for (int i = 0; i < vals.length; i++) {
                maxAt = vals[i] > vals[maxAt] ? i : maxAt;
            }

            channelIndexLast = maxAt;

            // Test code to get HSV vals

//            this.hsvVals = new Double[] {
//                    hMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]))[0],
//                    sMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]))[0],
//                    vMat.get((int)(input.rows()* midPos[1]), (int)(input.cols()* midPos[0]))[0]
//            };

            Imgproc.cvtColor(all, all, Imgproc.COLOR_HSV2RGB);
            return all;
            // return input;
        }

        public int getLastResult() {
            return channelIndexLast;
        }

//        public Double[] getHsvVals() {
//            return hsvVals;
//        }
    }
}
