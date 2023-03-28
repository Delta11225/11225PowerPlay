package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.types.AutoState;
import org.firstinspires.ftc.teamcode.util.types.AutoType;
import org.firstinspires.ftc.teamcode.util.types.Color;
import org.firstinspires.ftc.teamcode.util.types.StartPosition;

/**
 * This is a class that holds all necessary constants for the robot. Put "magic numbers" here.
 * This class cannot be instantiated. Do not inherit from it.
 * To add a new constant, simply declare a new public final static variable and assign the
 * appropriate value.
 */
public abstract class Constants {
    // Set this to true before competition. When true, stops ignoring not found hardware in the hardware file.
    public final static boolean COMPETITION_MODE = true;

    // Speed multipliers for teleop
    public final static double fastMultiplier = 0.8;
    public final static double normalMultiplier = 0.4;
    public final static double slowMultiplier = 0.3;
    public final static double superSlowMultiplier = 0.2;

    // Various values for the claw
    // right claw is plugged into port 0, left into 1
    public final static double rightClawOpen = 0.64;
    public final static double rightClawClosed = 0.53;
    public final static double leftClawOpen = 0.48;
    public final static double leftClawClosed = 0.6;

    public final static double liftUpPower = 1;
    public final static double liftDownPower = 1;
    public final static double liftPosRunPower = 1;

    // How much to change linear linear target when lift or lower buttons are pressed
    public final static int upEncoderStep = 30;
    public final static int downEncoderStep = 30;

    // Stores zero offset, if there is one
    public static int linearSlideZeroOffset = 0;

    private final static int liftEncoderMax = 4320;

    // Set at end of auto for teleop to use
    public static Pose2d currentPose = new Pose2d();
    public static AutoState matchState = new AutoState(Color.BLUE, StartPosition.FRONT, AutoType.LONG, 0);

    // Minimum distance cone can be away from sensor to autograb
    public static double minAutoGrabDistance = 3;

    // Gets various linear slide heights, and takes zero offset into account
    public static int getLiftEncoderMax() {
            return liftEncoderMax + linearSlideZeroOffset;
    }
    //    private final static int[] liftEncoderJunctions = new int[] {1780, -1, -1, -1};
    public static int[] getLiftEncoderJunctions() {
        return new int[] {
                1630 + linearSlideZeroOffset, // 0 = low junction
                2780 + linearSlideZeroOffset, // 1 = med junction
                3930 + linearSlideZeroOffset, // 2 = high junction
                liftEncoderMax + linearSlideZeroOffset
        };
    }

    // Gets the cone stack heights and takes zeroOffset into account
    private final static int[] liftEncoderConeStack = new int[]{603, 465, 330, 161, 0};
    public static int[] getLiftEncoderConeStack() {
        return new int[] {
                liftEncoderConeStack[0] + linearSlideZeroOffset,
                liftEncoderConeStack[1] + linearSlideZeroOffset,
                liftEncoderConeStack[2] + linearSlideZeroOffset,
                liftEncoderConeStack[3] + linearSlideZeroOffset,
                liftEncoderConeStack[4] + linearSlideZeroOffset
        };
    }

    // Maximum amount robot can be tilted before tilt correction engages
    public static final double maxTiltDegrees = 7;

    // The point at which it will stop correcting as we are on the ground and further movement would damage the robot
    public static final double unrecoverableTiltDegrees = 60;

    // expTotalScale is C, expAngleScale is A, in equation y=Ce^Ax, where x is angle diff
    public static final double expTotalScale = .06;
    public static final double expAngleScale = .32;

    // How long to wait to autograb after open button is pressed
    public static final double autoGrabCooldownSeconds = 2;
}