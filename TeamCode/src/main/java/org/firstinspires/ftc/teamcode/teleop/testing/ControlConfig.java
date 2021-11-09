package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.hardware.Gamepad;

/*
This class handles control dispatching. Simply access any of these variables to get the
status of that action's corresponding control.
DO NOT instantiate this class. ALL VARIABLES ARE STATIC.
AT THE BEGINNING of the teleop loop call this line of code: ControlConfig.update(gamepad1, gamepad2);
IF YOU DON'T DO THIS THE CONTROLS WILL NOT BE REFRESHED.
Follow given controls to add extra actions.
 */
public final class ControlConfig {
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

    public static boolean liftBucket;
    public static boolean lowerBucket;

    public static boolean duckWheelForward;
    public static boolean duckWheelBackward;

    public static boolean dumpServo;
    public static boolean collectServo;

    public ControlConfig() throws UnsupportedOperationException {
        throw new UnsupportedOperationException("I told you not to instantiate this. READ THE COMMENT");
    }

    public static void update(Gamepad pad1, Gamepad pad2) {
        // Update movement controls;
        forward = pad1.left_stick_y;
        backward = -pad1.left_stick_y;
        right = pad1.left_stick_x;
        left = -pad1.left_stick_x;
        clockwise = pad1.right_stick_x;

        fast = pad1.right_trigger > .1;
        slow = pad1.left_trigger > .1;

        // Update peripheral controls
        collectWheel = pad1.right_bumper;
        unCollectWheel = pad1.left_bumper;

        liftBucket = pad2.dpad_up;
        lowerBucket = pad2.dpad_down;

        duckWheelForward = pad2.right_bumper;
        duckWheelBackward = pad2.left_bumper;

        dumpServo = pad2.y;
        collectServo = pad2.a;
    }
}
