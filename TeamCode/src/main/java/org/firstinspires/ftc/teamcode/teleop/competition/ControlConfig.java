package org.firstinspires.ftc.teamcode.teleop.competition;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * This class handles control dispatching. Simply access any of these variables to get the
 * status of that action's corresponding control.
 * DO NOT instantiate this class. It won't let you. All needed variables and methods are static.
 * Also do not INHERIT FROM this class.
 * AT THE BEGINNING of the teleop loop call this line of code: ControlConfig.update(gamepad1, gamepad2);
 * IF YOU DON'T DO THIS THE CONTROLS WILL NOT BE REFRESHED.
 * To add an extra action:
 * * Define a public static class variable with the appropriate data type
 * * In the update method, assign to the class variable the appropriate data
 */
public abstract class ControlConfig {
    // Movement
    public static double forward;
    public static double backward;
    public static double right;
    public static double left;
    public static double clockwise;

    public static boolean slow;
    public static boolean fast;

    // Peripherals
    public static boolean collectWheel;
    public static boolean unCollectWheel;

    public static boolean liftSlide;
    public static boolean lowerSlide;
    public static boolean runSlideToLowDump;

    public static boolean linearSlideOverride;

    public static boolean duckWheelBlue;
    public static boolean duckWheelRed;

    public static boolean dumpBucket;
    public static boolean collectBucket;

//    public static boolean toggleTseRodServo;
//    public static boolean toggleTseArmServo;
    public static boolean raiseTseArm;
    public static boolean lowerTseArm;
    public static boolean initTseArm;
    public static boolean collectTseArm;

    public static boolean linSlideSlow;

    public static boolean playMotivSound;

    public static void update(Gamepad pad1, Gamepad pad2) {
        // Update movement controls;
        forward = -pad1.left_stick_y;
        backward = pad1.left_stick_y;
        right = pad1.left_stick_x;
        left = -pad1.left_stick_x;
        clockwise = pad1.right_stick_x;
//        clockwise = pad1.right_stick_y;

        fast = pad1.left_bumper;
        slow = pad1.right_bumper;

        // Update peripheral controls
        collectWheel = pad1.left_trigger > .1;
        unCollectWheel = pad1.right_trigger > .1;

        lowerTseArm = pad1.a;
        raiseTseArm = pad1.y;
        initTseArm = pad1.x;
        collectTseArm = pad1.b;

        liftSlide = pad2.dpad_up;
        lowerSlide = pad2.dpad_down;
        linSlideSlow = pad2.right_trigger > .1;

        duckWheelBlue = pad2.right_bumper;
        duckWheelRed = pad2.right_bumper;

        dumpBucket = pad2.y;
        collectBucket = pad2.a;

        linearSlideOverride = pad2.left_bumper;

        playMotivSound = pad1.left_stick_button || pad1.right_stick_button || pad2.left_stick_button || pad2.right_stick_button;
    }
}