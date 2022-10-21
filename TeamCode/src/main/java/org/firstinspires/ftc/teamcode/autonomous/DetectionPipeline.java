package org.firstinspires.ftc.teamcode.autonomous;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class DetectionPipeline extends OpenCvPipeline {
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
    Mat ones = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // TODO resize image so the center of the image is bigger
        // Do HSV stuff, crank S and V
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(hsvMat, hMat, 0);
        // We don't need these channels
//            Core.extractChannel(hsvMat, sMat, 1);
//            Core.extractChannel(hsvMat, vMat, 2);

        // Get mat of all ones of the same size as input to set saturation and value (brightness)
        // to max
        ones = new Mat(input.rows(), input.cols(), hMat.type(), new Scalar(255));

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
        Imgproc.circle(displayMat, pointMid,7, new Scalar( 0, 0, 0 ),3 );//draws circle

        // Draw rectangle around point
        // Circle and rectangle ware just for the user, they help with alignment
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

        // A mapping between channel index (r,g,b) and position (1,2,3). Change it if signal sleeve
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