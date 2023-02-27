package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.autonomous.AutoState;
import org.firstinspires.ftc.teamcode.autonomous.AutoType;
import org.firstinspires.ftc.teamcode.autonomous.Color;
import org.firstinspires.ftc.teamcode.autonomous.StartPosition;

/**
 * This is a class that holds all necessary constants for the robot. Put "magic numbers" here.
 * This class cannot be instantiated. Do not inherit from it.
 * To add a new constant, simply declare a new public final static variable and assign the
 * appropriate value.
 */
public abstract class Constants {
    // Set this to true before competition. When true, stops ignoring null motors in the hardware
    // file.
    public final static boolean COMPETITION_MODE = false;

    public final static double fastMultiplier = 0.8;
    public final static double normalMultiplier = 0.4;
    public final static double slowMultiplier = 0.3;
    public final static double superSlowMultiplier = 0.2;

    public final static int motivationQuantity = 4;

    public final static int autoLiftEncoderStart = 40;

    public final static double leftClawOpen = 0.48;
    public final static double leftClawClosed = 0.6;
    //right claw is plugged into port 0, left into 1
    public final static double rightClawOpen = 0.64;
    public final static double rightClawClosed = 0.53;

    public final static double liftUpPower = 1;
    public final static double liftDownPower = 1;
    public final static double liftPosRunPower = 1;

    public final static int upEncoderStep = 30;
    public final static int downEncoderStep = 30;

    public static int linearSlideZeroOffset = 0;

    private final static int liftEncoderMax = 4530;

    // Set at end of auto for teleop to sue
    public static Pose2d currentPose = new Pose2d();
    public static AutoState matchState = new AutoState(Color.BLUE, StartPosition.FRONT, AutoType.LONG, 0);
    public static double minAutoGrabDistance = 3;

    public static int getLiftEncoderMax() {
            return liftEncoderMax + linearSlideZeroOffset;
    }
    //    private final static int[] liftEncoderJunctions = new int[] {1780, -1, -1, -1};
    public static int[] getLiftEncoderJunctions() {
        return new int[] {
                1750 + linearSlideZeroOffset,
                2910 + linearSlideZeroOffset,
                4005 + linearSlideZeroOffset,
                liftEncoderMax + linearSlideZeroOffset
        };
    }
//    public final static int liftEncoderLow = 1780;

    private final static int[] liftEncoderConeStack = new int[]{650, 534, 369, 181, 0};
    public static int[] getLiftEncoderConeStack() {
        return new int[] {
                liftEncoderConeStack[0] + linearSlideZeroOffset,
                liftEncoderConeStack[1] + linearSlideZeroOffset,
                liftEncoderConeStack[2] + linearSlideZeroOffset,
                liftEncoderConeStack[3] + linearSlideZeroOffset,
                liftEncoderConeStack[4] + linearSlideZeroOffset
        };
    }

    public static final double maxTiltDegrees = 7;

    // The point at which it will stop correcting as we are on the ground and further movement would damage the robot
    public static final double unrecoverableTiltDegrees = 60;

    // Deals with the logarithmic function that controls how aggressively we correct tilt. In a logistic
    // growth function C log Ax, logisticScale is the C and valueScale is the A.
    public static final double tiltCorrectionLogisticScale = 0.5;
    public static final double tiltCorrectionValueScale = 2;

    // expTotalScale is C, expAngleScale is A, in equation y=Ce^Ax, where x is angle diff
    public static final double expTotalScale = .06;
    public static final double expAngleScale = .32;

    public static final double autoGrabCooldownSeconds = 2;
}