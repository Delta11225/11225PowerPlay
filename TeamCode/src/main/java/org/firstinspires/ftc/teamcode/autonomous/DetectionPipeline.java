package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.util.types.ParkingPosition;
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
    // The point where we actuall want to detect, in terms of fractions of width and height.
    // So 1/2, 1/2 is halfway across, halfway up, or in the center
    private static final float[] detectionPoint = {1f/2f, 1f/2f};

    // The width and height of the rectangle around the detection point, only for display. Does not
    // affect detection
    private static final float rectHeight = 1f/8f;
    private static final float rectWidth =  1f/8f;

    // These are storage variables for the last detected values. channelIndex is the index of the
    // highest channel, rawColorVals are the raw values detected by the camera, and posLast is
    // the last parking position. They are initialized to some value to prevent possible null
    // errors down the line.
    private int channelIndexLast = -1;
    private double[] rawColorVals = new double[] {-1d, -1d, -1d};
    private ParkingPosition posLast = ParkingPosition.HOW_ON_EARTH;

    // displayMat stores the mat that will actually be returned to the user and displayed on the
    // driver hub
    Mat displayMat = new Mat();

    // hsvMat stores the result of converting the input image mat into HSV, as we want to saturate
    // the image to make color changes more obvious and reduce noise
    Mat hsvMat = new Mat();
    // This stores the hue channel
    Mat hMat = new Mat(); // Channel 0
    // These mats are not needed, as they will be replaced with mats of just ones to saturate the image
//            Mat sMat = new Mat(); // Channel 1
//            Mat vMat = new Mat(); // Channel 2

    // satMat stores the result of hMat saturated and converted back to RGB
    Mat satMat = new Mat();
    // This mat is just a whole bunch of ones used to replace the sMat and vMat.
    Mat ones = new Mat();

    // This list will store mats that will then be merged back into a single mat after saturation
    List<Mat> matList;

    /**
     * This method runs every frame, and determines how the frame is processed.
     * @param input The webcam frame. Provided by the opmode
     * @return The image that is displayed back to the user.
     */
    @Override
    public Mat processFrame(Mat input) {
        // Convert to HSV to make saturating the image easier
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.extractChannel(hsvMat, hMat, 0);
        // We don't need these channels
//            Core.extractChannel(hsvMat, sMat, 1);
//            Core.extractChannel(hsvMat, vMat, 2);

        // Get mat of all ones of the same size as input to set saturation and value (brightness)
        // to max
        ones = new Mat(input.rows(), input.cols(), hMat.type(), new Scalar(255));

        // Put the mats in the proper order as merge() expects a list
        matList = Arrays.asList(
                hMat,
                ones,
                ones
        );

        Core.merge(matList, satMat);
//            satMat.copyTo(all);

        // Convert back to RGB as the driver hub expects an image in RGB format
        Imgproc.cvtColor(satMat, displayMat, Imgproc.COLOR_HSV2RGB);

        // Create point that represents detection point
        Point pointMid = new Point((int)(input.cols()* detectionPoint[0]), (int)(input.rows()* detectionPoint[1]));

        // Draw circle on specified detection point to aid user
        Imgproc.circle(displayMat, pointMid,7, new Scalar( 0, 0, 0 ),3 );

        // Draw rectangle around point
        // Circle and rectangle are just for the user, they help with alignment but don't affect detection
        Imgproc.rectangle(
                displayMat,
                new Point(
                        input.cols()*(detectionPoint[0]- rectWidth /2),
                        input.rows()*(detectionPoint[1]- rectHeight /2)),
                new Point(
                        input.cols()*(detectionPoint[0]+ rectWidth /2),
                        input.rows()*(detectionPoint[1]+ rectHeight /2)),
                new Scalar(0, 255, 0), 3);

        // Get channel index with maximum value to find what color is the brightest. We want to see
        // whether the detection point is red, green, or blue, and *conveniently*, the image is
        // reported in RGB (red, green, blue) format. Almost like we planned it like that.
        // The index of the channel with the maximum value probably corresponds to which color is
        // brightest.

        // There are probably easier ways to do this with hue distance or something, but
        // I am too lazy to even try to implement those. This is easy and it works
        double[] vals = displayMat.get((int)(input.rows()* detectionPoint[1]), (int)(input.cols()* detectionPoint[0]));

        int maxAt = 0;

        // Loop through colors and find index with maximum value
        for (int i = 0; i < vals.length; i++) {
            maxAt = vals[i] > vals[maxAt] ? i : maxAt;
        }

        // Store for later
        channelIndexLast = maxAt;

        // A mapping between channel index (r,g,b) and parking position (1,2,3). Change it if
        // signal sleeve changes.
        HashMap<Integer, ParkingPosition> posMappings = new HashMap<>();
        posMappings.put(0, ParkingPosition.THREE);
        posMappings.put(1, ParkingPosition.ONE);
        posMappings.put(2, ParkingPosition.TWO);

        // Set last parking position according to mapping
        posLast = posMappings.getOrDefault(maxAt, ParkingPosition.HOW_ON_EARTH);

        // Set these for debugging
        rawColorVals = vals;

        // If we don't do this, we get a memory leak, so we make sure to release the mats after we
        // no longer need to use them.
        hsvMat.release();
        hMat.release();

        satMat.release();
        ones.release();

        return displayMat;
    }

    public int getLastChannelMax() {
        return channelIndexLast;
    }

    public double[] getRawColorVals() {
        return rawColorVals;
    }

    public ParkingPosition getLastPos() {
        return posLast;
    }
}