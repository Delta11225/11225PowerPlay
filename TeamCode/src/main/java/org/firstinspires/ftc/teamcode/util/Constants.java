package org.firstinspires.ftc.teamcode.util;

/**
 * This is a class that holds all necessary constants for the robot. Put "magic numbers" here.
 * This class cannot be instantiated. Do not inherit from it.
 * To add a new constant, simply declare a new public final static variable and assign the
 * appropriate value.
 */
public abstract class Constants {
    // TODO remove all the opmodes that depend on these constants
    public final static double dumpPosition = 0.0;
    public final static double collectPosition = 0.73;
    public final static double holdPosition = 0.35;

    public final static double tseArmInitPosition = 0.47;
    public final static double tseArmCollectPosition = 0.83;
    public final static double tseArmActivePosition = 0.46;
//
//    public final static double tseArmCollectPosition = 0.40;
    public final static double tseArmDeployPosition = 0.2;
    public final static double tseArmReleasePosition = 0.3;

    public final static double fastMultiplier = 0.8;
    public final static double normalMultiplier = 0.4;
    public final static double slowMultiplier = 0.3;
    public final static double superSlowMultiplier = 0.2;

    public final static double towerWheelSpeedEndgame = .6009;
    public final static double towerWheelSpeedAuto = .420;
    public final static double duckAccelIncrement = 0.05;

    public final static double tseStep = 0.0005;

    public final static int lowEncoder = 1120;
    public final static int midEncoder = 2270;
    public final static int highEncoder = 3470;

    public final static double lowestDump = 1100;
    public final static int motivationQuantity = 4;

    public final static int autoLiftEncoderStart = 40;

    public final static double rightClawOpen = 0.90;
    public final static double rightClawClosed = 0.7;

    public final static double leftClawOpen = 0.05;
    public final static double leftClawClosed = 0.25;

    public final static double liftUpPower = 0.75;
    public final static double liftDownPower = 0.5;
    public final static double liftPosRunPower = 1;

    public final static int upEncoderStep = 10;
    public final static int downEncoderStep = 10;

    public static int linearSlideZeroOffset = 0;

    private final static int liftEncoderMax = 4500;
    public static int getLiftEncoderMax() {
            return liftEncoderMax + linearSlideZeroOffset;
    }
    //    private final static int[] liftEncoderJunctions = new int[] {1780, -1, -1, -1};
    public static int[] getLiftEncoderJunctions() {
        return new int[] {
                1780 + linearSlideZeroOffset,
                2930 + linearSlideZeroOffset,
                4000 + linearSlideZeroOffset,
                liftEncoderMax + linearSlideZeroOffset
        };
    }
//    public final static int liftEncoderLow = 1780;

    public final static int[] liftEncoderConeStack = new int[]{706, 495, 347, 181, 0};
    public static int[] getLiftEncoderConeStack() {
        return new int[] {
                706 + linearSlideZeroOffset,
                495 + linearSlideZeroOffset,
                347 + linearSlideZeroOffset,
                181 + linearSlideZeroOffset,
                0 + linearSlideZeroOffset
        };
    }
}