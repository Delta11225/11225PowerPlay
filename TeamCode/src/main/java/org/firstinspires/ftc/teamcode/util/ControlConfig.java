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

    // Movement speed modifiers
    public static boolean slow = false;
    public static boolean fast = false;

    // Peripherals
    // Linear slide manual movement
    public static boolean liftSlide = false;
    public static boolean lowerSlide = false;

    // Linear slide automated movement
    public static boolean goToGround = false;
    public static boolean goToLow = false;
    public static boolean goToMedium = false;
    public static boolean goToHigh = false;

    // Claw
    public static boolean openClaw = false;
    public static boolean closeClaw = false;

    // Misc controls
    public static boolean overrideModifier = false;
    // We don't really use this.
    public static boolean resetIMU = false;

    /**
     * Updates controls. Contains the mapping for all controls. Should be called at the start of teleop
     * loop to refresh controls
     * @param pad1 The gamepad1 object provided by the opmode
     * @param pad2 The gamepad2 object provided by the opmode
     */
    public static void update(Gamepad pad1, Gamepad pad2) {
        // Update movement values. Currently set so that field 270 (facing drivers) is forward
        if (pad2.touchpad_finger_1) {
            forward = pad2.touchpad_finger_1_y;
            right = pad2.touchpad_finger_1_x;
        } else {
            forward = 0;
            right = 0;
        }
        backward = -forward;
        left = -right;
        clockwise = pad1.right_stick_x;

        // Speed modifier values
        fast = pad1.left_bumper;
        slow = pad1.right_bumper;

        // Linear slide manual values
        liftSlide = pad2.dpad_up;
        lowerSlide = pad2.dpad_down;

        // Linear slide automated controls. Have or statement to handle ps4 dualshock and logitech.
        // Probably don't need, but just to be safe
        goToGround = pad2.cross || pad2.a;
        goToLow = pad2.square || pad2.x;
        goToMedium = pad2.triangle || pad2.y;
        goToHigh = pad2.circle || pad2.b;

        // Claw values
        openClaw = pad2.right_bumper;
        closeClaw = pad2.left_bumper;

        // Misc values
        overrideModifier = pad2.left_stick_button;
        resetIMU = pad1.left_stick_button && pad1.right_stick_button;
    }
}