package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.exception.MathArithmeticException;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.util.types.Color;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.ControlConfig;
import org.firstinspires.ftc.teamcode.util.Hardware23;
import org.firstinspires.ftc.teamcode.util.types.LinearSlideMode;

import java.util.Locale;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

//@Disabled
@TeleOp
public class TeleopFinal extends OpMode {
    // FIXME in the future reduce number of global vars
    Hardware23 robot;

    BNO055IMU imu;
    Orientation angles;

    double frontLeft;
    double rearLeft;
    double frontRight;
    double rearRight;

    double forward;
    double right;
    double clockwise;
    // Stores by how much we need to multiply movement controls by
    double powerMultiplier = 1;

    // For driver oriented control
    double temp;
    double side;
    double currentAngle;

    private LinearSlideMode linearSlideMode = LinearSlideMode.MANUAL;
    // Linear slide is in run to position mode, and this keeps track of where the slide should be going
    private int linearSlideTarget = 0;

    private boolean isClawClosed = false;
    private Color currentColor;

    // For the autograb cooldown. Helps prevent autograb from activating immediately after user
    // drops cone
    private final ElapsedTime lastAutoGrab = new ElapsedTime();

    /**
     * Runs once when init button is pressed
     */
    @Override
    public void init() {
        resetRuntime();
        robot = new Hardware23(hardwareMap);

        // Reset linear slide offset, as we reinit the linear slide
        Constants.linearSlideZeroOffset = 0;

        initIMU();
        telemetry.addData("Hardware init time", getRuntime());
        telemetry.update();

        // Get color of match as we need it for auto grab
        currentColor = Constants.matchState.color;

        // Stop all motors just in case
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.rearLeft.setPower(0);
        robot.rearRight.setPower(0);

        // Set appropriate motor directions
        robot.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.rearLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.rearRight.setDirection(DcMotor.Direction.REVERSE);

        // Initialize linear slide
        robot.linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Need to set target position before we set run to position mode
        robot.linearSlide.setTargetPosition(linearSlideTarget);
        robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addLine("Init complete, ready to run");
        telemetry.update();
    }

    /**
     * Runs once at the start of teleop
     */
    @Override
    public void start() {
        // Tell the IMU to start keeping track of whatever it needs to
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        // Reset current angle
        currentAngle = 0;

        resetRuntime();
    }

    /**
     * Initialize IMU with proper parameters
     */
    private void initIMU() {
        // IDK I copied this stuff from last year here's my best guess

        // Create IMU parameters object
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        // Set appropriate units for IMU reporting. We want to report in degrees as default as it is easier
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        // Black magic
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode

        // Make sure to log stuff from IMU and set logging tag so we know it is from the IMU
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        // Which black magic algorithm to use
        // This one seems to just log acceleration and nothing else
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Actually get the IMU and initalize it
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    /**
     * Runs constantly, in a loop, as long as teleop is active
     */
    @Override
    public void loop() {
        // Every time we loop, we need to update the controls as this class handles our control mapping
        ControlConfig.update(gamepad1, gamepad2);

        telemetry.update();
        // Robot movement (driver oriented control)
        move();
        // Other movement (stuff like linear slide)
        peripheralMove();

        // Update current angle, and prevent it from being negative
        if (angles.firstAngle < 0) {
            currentAngle = angles.firstAngle + 360;
        } else {
            currentAngle = angles.firstAngle;
        }
    }

    /**
     * Handle robot movement (driver oriented control)
     */
    private void move() {
        // Just in case
        ControlConfig.update(gamepad1, gamepad2);

        // Deals with tilt. Prevents robot from tilting too far. If above certain tilt angle, corrects it
        // and locks driver control.
        Vector3D robotNormalVec = getRobotNormalVector();
        // If we are past a certain tilt angle, we do not want to try to correct as it may cause
        // damage to the robot and field
        if (isOverMaxTilt(robotNormalVec) && isBelowUnrecoverableTilt(robotNormalVec)) {
            handleTilt(robotNormalVec);
            return;
        }

        // Get current angle in radians as inbuilt trig functions take radians
        double theta = Math.toRadians(currentAngle);

//        telemetry.addData("CurrentAngle", currentAngle);
//        telemetry.addData("Theta", theta);

        // FIXME if possible make it so that it works with any orientation
        // Get raw, unrotated values from user
        forward = ControlConfig.forward;
        right = ControlConfig.right;
        clockwise = ControlConfig.clockwise;

        // Perform a cartesian rotation to map driver plane to robot plane
        temp = (forward * Math.cos(theta) - right * Math.sin(theta));
        side = (forward * Math.sin(theta) + right * Math.cos(theta));

        forward = temp;
        right = side;

//        telemetry.addData("right: ", right);
//        telemetry.addData("forward: ", forward);
//        telemetry.addData("temp: ", temp);
//        telemetry.addData("side: ", side);
//        telemetry.addData("clockwise: ", clockwise);

        // Find actual motor power based off mechanum control
        frontLeft = forward + right + clockwise;
        rearLeft = forward - right + clockwise;
        rearRight = forward + right - clockwise;
        frontRight = forward - right - clockwise;

//        telemetry.addData("front left: ", frontLeft);
//        telemetry.addData("rear left: ", rearLeft);
//        telemetry.addData("rear right: ", rearRight);
//        telemetry.addData("front right: ", frontRight);

        // Handle speed control
        if (ControlConfig.slow && ControlConfig.fast) {
            powerMultiplier = Constants.superSlowMultiplier;
        } else if (ControlConfig.fast) {
            // Prevent robot from moving fast when linear slide is up. Avoids dangerous tilt issues when
            // robot suddenly stops.
            if (robot.linearSlide.getCurrentPosition() < Constants.getLiftEncoderJunctions()[0] + 10) {
                powerMultiplier = Constants.fastMultiplier;
            } else {
                powerMultiplier = Constants.normalMultiplier;
            }
        } else if (ControlConfig.slow) {
            powerMultiplier = Constants.slowMultiplier;
        } else {
            powerMultiplier = Constants.normalMultiplier;
        }

        telemetry.addData("Power:", powerMultiplier);

        telemetry.update();

        // Make sure to multiply motor powers by the power multipler to account for speed
        robot.frontLeft.setPower(frontLeft * powerMultiplier);
        robot.frontRight.setPower(-frontRight * powerMultiplier);
        robot.rearLeft.setPower(rearLeft * powerMultiplier);
        robot.rearRight.setPower(-rearRight * powerMultiplier);
    }

    /**
     * Calculates the normal vector of the robot based on reported IMU angles
     * @return The robot's normal vector
     */
    private Vector3D getRobotNormalVector() {
        // We already have angles from IMU for telemetry and driver oriented control, so we can just get that
        // It is in the order of ZYX

        // Angles are reported in degrees, but we need radians because that is what the trig functions take
        double x = Math.toRadians(angles.thirdAngle);
        double y = Math.toRadians(angles.secondAngle);
        double z = Math.toRadians(angles.firstAngle);

//        telemetry.addData("Angle 1 (x)", x);
//        telemetry.addData("Angle 2 (y)", y);
//        telemetry.addData("Angle 3 (z)", z);

//        Log.d("Tilt", String.format("Angle 1 (x) %f", x));
//        Log.d("Tilt", String.format("Angle 2 (y) %f", y));
//        Log.d("Tilt", String.format("Angle 3 (z) %f", z));

        // All this math just gets the robot's normal vector. In technical terms, it rotates a unit
        // z vector using roll (x), pitch (y), and yaw/bank/heading (z) angles. Refer to the following
        // website at the bottom of the answer for the rotation matrix used.
        // https://math.stackexchange.com/questions/1637464/find-unit-vector-given-roll-pitch-and-yaw

        Vector3D normalVec = new Vector3D(
                sin(x) * cos(y) * cos(z) + sin(y) * sin(z),
                sin(y) * cos(z) - sin(x) * cos(y) * sin(z),
                cos(x) * cos(y)
        );
        // Above should be normal, but just in case.
        try {
            normalVec = normalVec.normalize();
        } catch (MathArithmeticException e) {
            // Normalizing vectors of length 0 throws an error so we just ignore it if it happens
        }

        telemetry.update();

        return normalVec;
    }

    /**
     * Calculates the angle of the given normal vector to the Z axis
     * @param robotNormalVec The vector to find the angle of
     * @return The angle of the given vector to the Z axis
     */
    private double calcTiltAngle(Vector3D robotNormalVec) {
        // To compute the angle to the Z axis (tilt angle) we need arccos of the the dot product
        // with the given vector and the Z axis. Fortunately the dot product simplifies down to
        // this single term since two of the components of the z unit vector are zero (0, 0, 1)
        return Math.acos(robotNormalVec.getZ());
    }

    /**
     * Given the robot's normal vector, determines whether the robot has exceeded acceptable tilt
     * @param normalVec The robot's normal vector
     * @return Whether the robot is tilting too much
     */
    private boolean isOverMaxTilt(Vector3D normalVec) {
        double angleToZ = calcTiltAngle(normalVec);

        // Need to take absolute value in case angle is negative
        return Math.abs(Math.toDegrees(angleToZ)) >= Constants.maxTiltDegrees;
    }

    /**
     * Calculates whether the robot has tilted past an unrecoverable state
     * @param normalVec The robot's normal vector
     * @return Whether the vector is tilted too far
     */
    private boolean isBelowUnrecoverableTilt(Vector3D normalVec) {
        double angleToZ = calcTiltAngle(normalVec);

        return Math.abs(Math.toDegrees(angleToZ)) <= Constants.unrecoverableTiltDegrees;
    }

    /**
     * Handle all movement control if the robot is tilted too far
     * @param robotNormalVec The normal vector of the robot
     */
    private void handleTilt(Vector3D robotNormalVec) {
        // Find the difference between the robot's tilt angle and the maximum acceptable tilt
        double angleDiff = Math.abs(Math.toDegrees(calcTiltAngle(robotNormalVec)) - Constants.maxTiltDegrees);
        // Project the vector onto the XY plane to find direction robot should move
        Vector2D responseVec = getNormalAxisProjection(robotNormalVec);

        // Correct based on how far off the axis we are. We use an exponential correction factor to
        // correct slowly at first and fast as angle diff increases
        double correctionFactor = calcExponentialCorrectionFactor(angleDiff);
        responseVec = responseVec.scalarMultiply(correctionFactor);

        Log.d("TiltCorr", String.format("Angle diff %f", (angleDiff)));
        Log.d("TiltCorr", String.format("Correction factor %f", correctionFactor));

        double responseX = responseVec.getX();
        double responseY = responseVec.getY();
        telemetry.addData("Response vec x", responseX);
        telemetry.addData("Response vec y", responseY);
        telemetry.update();

        Log.d("TiltCorr", String.format("Response vec x %f", responseX));
        Log.d("TiltCorr", String.format("Response vec y %f", responseY));

        // Motor power has to be set in a specific and annoying way since 2 motors are off the ground
        setTiltMotorPower(responseVec);
    }

    /**
     * Find the normalized projection of the given vector onto the XY plane
     * @param normalVec The vector to find the projection of
     * @return The normalized projection onto the XY plane
     */
    private Vector2D getNormalAxisProjection(Vector3D normalVec) {
        // The projection of a vector onto the XY axis is just the X and Y components. We don't normalize
        return new Vector2D(normalVec.getX(), normalVec.getY()).normalize();
    }

    /**
     * Set the motor power to correct tilt based off the response vector if the robot is tilted.
     * @param responseVec The vector to set motor power based off
     */
    private void setTiltMotorPower(Vector2D responseVec) {
        // Because two motors are off the ground, the powers of the motors have to be set in a specific
        // way, and this method handles all of that.

        // For some reason, as it is now, this causes  the robot to spin when correcting,
        // but I am too lazy to try to fix. It's not a bug, it's a feature, I guess.
        double x = responseVec.getX();
        double y = responseVec.getY();
        double frontLeftPower = y - x;
        double rearLeftPower = y - x;
        double rearRightPower = -y - x;
        double frontRightPower = x + y;
        robot.drive.setMotorPowers(frontLeftPower, rearLeftPower, rearRightPower, frontRightPower);
    }

    /**
     * Find the correction factor based off of an exponential scaling function
     * @param angleDiff The angle difference to scale by
     * @return The calculated correction factor
     */
    private double calcExponentialCorrectionFactor(double angleDiff) {
        return Constants.expTotalScale * Math.pow(Math.E, Constants.expAngleScale * Math.abs(angleDiff));
    }

    /**
     * Handle all non-wheel movement (linear slide, etc)
     */
    private void peripheralMove() {
        // Just to be sage
        ControlConfig.update(gamepad1, gamepad2);

        // Extracted into different methods to make this method cleaner
        linearSlideMoveWithOverride();
        handleClaw();
        handleClawAutoGrab();
    }

    /**
     * Move the linear slide, and take overrides into account.
     */
    private void linearSlideMoveWithOverride() {
        // We refer to it a lot, so we fetch it here for easy use
        DcMotor linearSlide = robot.linearSlide;

        telemetry.update();

        // Complex if statement, but if we want to lift the lide AND the linear slide is below the max
        // AND we are not overriding, lift the slide
        if (ControlConfig.liftSlide && linearSlide.getCurrentPosition() < Constants.getLiftEncoderMax() && !ControlConfig.overrideModifier) {
            // Reset linear slide mode to manual as we want to interrupt movement
            linearSlideMode = LinearSlideMode.MANUAL;

            // Set the target to the MINIMUM of either the current target plus the up encoder counts or the
            // linear slide max to prevent linear slide from going too high
            // We add encoder counts to linear slide target to set the target higher and make the
            // linear slide go up
            linearSlideTarget = Math.min(linearSlideTarget + Constants.upEncoderStep, Constants.getLiftEncoderMax());

            // Same logic as above, just for going down. Make sure to use zeroOffset and not 0 since
            // we don't know where linear side 0 is (if it has been overriden) and zeroOffset should be
            // the minimum for the slide
        } else if (ControlConfig.lowerSlide && linearSlide.getCurrentPosition() > Constants.linearSlideZeroOffset && !ControlConfig.overrideModifier) {
            linearSlideMode = LinearSlideMode.MANUAL;

            // Same as the min above, just now with max and zeroOffset and we are subtracting.
            // Prevents slide from going down too far.
            linearSlideTarget = Math.max(linearSlideTarget - Constants.downEncoderStep, Constants.linearSlideZeroOffset);

            // If we are not overriding and in manual mode, set target to current position. Prevents
            // slide from moving after button is released.
        } else if (linearSlideMode == LinearSlideMode.MANUAL && !ControlConfig.overrideModifier) {
            linearSlideTarget = linearSlide.getCurrentPosition();
        }

        // Functionally, all these various if statements are the same, aside from the ground that
        // has a special safety. If we pushed the button to go to a certain location, set the target
        // to there and change the mode

        // Ella safety.
        // For the ground, we don't want to go to ground if the claw is closed and we are above
        // the low junction
        long linearSlidePos = robot.linearSlide.getCurrentPosition();
        long ellaSafetyThreshold = Constants.getLiftEncoderJunctions()[0] - 20;
        if (ControlConfig.goToGround && !(isClawClosed && linearSlidePos > ellaSafetyThreshold)) {
            linearSlideMode = LinearSlideMode.GROUND;
            linearSlideTarget = Constants.linearSlideZeroOffset;
        }

        if (ControlConfig.goToLow) {
            linearSlideMode = LinearSlideMode.LOW;
            linearSlideTarget = Constants.getLiftEncoderJunctions()[0];
        }

        if (ControlConfig.goToMedium) {
            linearSlideMode = LinearSlideMode.MEDIUM;
            linearSlideTarget = Constants.getLiftEncoderJunctions()[1];
        }

        if (ControlConfig.goToHigh) {
            linearSlideMode = LinearSlideMode.HIGH;
            linearSlideTarget = Constants.getLiftEncoderJunctions()[2];
        }

        // Handle zero overrides since we ignore them above. Some of the logic is reused and I will only
        // comment new logic

        // Overriding allows drivers to bypass the safety that prevents the user from lowering or
        // raising the slide too high in case the user needs to reset the linear slide's zero position

        // If we override and are lifting the slide
        if (ControlConfig.overrideModifier && ControlConfig.liftSlide) {
            // Reset back to manual mode just in case
            if (linearSlideMode != LinearSlideMode.MANUAL) {
                linearSlideMode = LinearSlideMode.MANUAL;
                linearSlideTarget = linearSlide.getCurrentPosition();
            }

            linearSlideTarget = linearSlideTarget + Constants.upEncoderStep;

            // Reset the zeroOffset to the current position, as that is supposedly the new 0.
            Constants.linearSlideZeroOffset = linearSlide.getCurrentPosition();
        }

        // Same as above.
        if (ControlConfig.overrideModifier && ControlConfig.lowerSlide) {
            if (linearSlideMode != LinearSlideMode.MANUAL) {
                linearSlideMode = LinearSlideMode.MANUAL;
                linearSlideTarget = linearSlide.getCurrentPosition();
            }

            linearSlideTarget = linearSlideTarget - Constants.upEncoderStep;

            Constants.linearSlideZeroOffset = linearSlide.getCurrentPosition();
        }

        // The linear slide is in run to position mode to prevent it from falling down when it is high
        // up. Here we set the target pos to whatever we have calculated it to be.
        robot.linearSlide.setTargetPosition(linearSlideTarget);
        // We shouldn't need to do this, but just in case
        robot.linearSlide.setPower(Constants.liftPosRunPower);
        telemetry.addData("Linear Slide target pos", linearSlideTarget);
//        Log.d("LinearSlide", String.valueOf(linearSlideTarget));
        telemetry.update();
    }

    /**
     * Handles claw control and movement
     */
    private void handleClaw() {
        // A button = open claw, b button = closed claw
        // Prevents weirdness with gamepad2 locking up
        gamepad2.toString();
        if (ControlConfig.openClaw) {
            isClawClosed = false;
            // Reset claw grab cooldown
            lastAutoGrab.reset();
            robot.rightClaw.setPosition(Constants.rightClawOpen); // Right claw open
            robot.leftClaw.setPosition(Constants.leftClawOpen); // Left claw open
        } else if (ControlConfig.closeClaw) {
            isClawClosed = true;
            robot.rightClaw.setPosition(Constants.rightClawClosed); // Right claw closed
            robot.leftClaw.setPosition(Constants.leftClawClosed); // Left claw closed
        }
    }

    /**
     * Deal with auto grabbing claw
     */
    private void handleClawAutoGrab() {
        // Prevent autograbbing at end of teleop so if we let go of a cone right as teleop ends
        // it doesn't grab it back
        if (getRuntime() >= 115) {
            return;
        }

        // If the cooldown has not expired, stop
        if (lastAutoGrab.seconds() < Constants.autoGrabCooldownSeconds) {
            return;
        }

        // Don't autograb if we are holding the button to open the claw
        if (ControlConfig.openClaw) {
            return;
        }

        // If the claw is closed, don't auto grab
        if (isClawClosed) {
            return;
        }

        // If the claw is above the low junction, don't auto grab
        if (robot.linearSlide.getCurrentPosition() > Constants.getLiftEncoderJunctions()[0] - 40) {
            return;
        }

        // Get color (red and blue) of and distance to cone.
        ColorSensor colorSensor = robot.colorSensor;
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

        // If we are too far from the cone, don't auto grab
        if (distance > Constants.minAutoGrabDistance) {
            return;
        }

        // Only grab the cone if the cone is the same color as our team
        if (currentColor == Color.BLUE ? blue > red : red > blue) {
            robot.rightClaw.setPosition(Constants.rightClawClosed);
            robot.leftClaw.setPosition(Constants.leftClawClosed);
            // Rumble peripheral controller to let user know auto grab has happened
            gamepad2.rumble(250);
            isClawClosed = true;
            // Rest claw auto grab cooldown
            lastAutoGrab.reset();
        }
    }

    /*-----------------------------------//
     * DO NOT WRITE CODE BELOW THIS LINE  *
     * -----------------------------------*/
    void composeTelemetry() {
        // At the beginning of each telemetry update, grab angles from IMU
        telemetry.addAction(() -> {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
