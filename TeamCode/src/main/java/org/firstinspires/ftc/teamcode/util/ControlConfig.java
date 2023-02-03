package org.firstinspires.ftc.teamcode.util;

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
    public static double forward = 0;
    public static double backward = 0;
    public static double right = 0;
    public static double left = 0;
    public static double clockwise = 0;

    public static boolean slow = false;
    public static boolean fast = false;

    // Peripherals
    public static boolean linSlideSlow = false;

    public static boolean playMotivSound = false;

    public static boolean openClaw = false;
    public static boolean closeClaw = false;

    public static boolean goToGround = false;
    public static boolean goToLow = false;
    public static boolean goToMedium = false;
    public static boolean goToHigh = false;

    public static boolean resetIMU = false;

    public static void update(Gamepad pad1, Gamepad pad2) {
        // Update movement controls. Currently set so that facing 270 is forward
        forward = pad1.left_stick_y;
        backward = -pad1.left_stick_y;
        right = -pad1.left_stick_x;
        left = pad1.left_stick_x;
        clockwise = pad1.right_stick_x;
//        clockwise = pad1.right_stick_y;

        fast = pad1.left_bumper;
        slow = pad1.right_bumper;

        liftSlide = pad2.dpad_up;
        lowerSlide = pad2.dpad_down;
        linSlideSlow = pad2.right_trigger > .1;

//        openClaw = pad2.a || pad2.cross;
//        closeClaw = pad2.b || pad2.circle;
        openClaw = pad2.right_bumper;
        closeClaw = pad2.left_bumper;

        goToGround = pad2.cross || pad2.a;
        goToLow = pad2.square || pad2.x;
        goToMedium = pad2.triangle || pad2.y;
        goToHigh = pad2.circle || pad2.b;

        overrideModifier = pad2.left_stick_button;

        resetIMU = pad1.left_stick_button && pad1.right_stick_button;
    }
}