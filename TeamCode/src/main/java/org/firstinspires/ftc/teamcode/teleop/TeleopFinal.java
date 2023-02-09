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
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.autonomous.Color;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.ControlConfig;
import org.firstinspires.ftc.teamcode.util.Hardware23;
import org.firstinspires.ftc.teamcode.util.LinearSlideMode;

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
    Acceleration gravity;

    double frontLeft;
    double rearLeft;
    double frontRight;
    double rearRight;

    double forward;
    double right;
    double clockwise;
    double powerMultiplier = 1;
    double deadZone = Math.abs(0.2);

    double temp;
    double side;
    double currentAngle;

    boolean motivated = false;
    boolean hasRumbled = false;
    boolean didRumble1 = false;

    double offset = 0;
    int holdPosition;
    ElapsedTime elapsedTime = new ElapsedTime();
    private final ElapsedTime runtime = new ElapsedTime();
//    private boolean runningToPos = false;
    private LinearSlideMode linearSlideMode = LinearSlideMode.MANUAL;
    private int linearSlideTarget = 0;
    private boolean areInittingIMU = false;

    private boolean isClawClosed = false;
    private Color currentColor;

    private ElapsedTime lastAutoGrab = new ElapsedTime();

    @Override
    public void init() {
        resetRuntime();

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Constants.linearSlideZeroOffset = 0;
        currentColor = Constants.matchState.color;

        initIMU();
        areInittingIMU = false;
        telemetry.addData("IMU init time", getRuntime());
        telemetry.update();

        resetRuntime();
        robot = new Hardware23(hardwareMap);

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.rearLeft.setPower(0);
        robot.rearRight.setPower(0);

        robot.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.rearLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.rearRight.setDirection(DcMotor.Direction.REVERSE);

        robot.linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearSlide.setTargetPosition(linearSlideTarget);
        robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Robot HWMap init time", getRuntime());
        telemetry.update();
    }

    @Override
    public void start() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        currentAngle = 0;

        elapsedTime.reset();
        resetRuntime();
    }

    @Override
    public void loop() {
        ControlConfig.update(gamepad1, gamepad2);

        telemetry.update();
        move();
        peripheralMove();

        if (angles.firstAngle < 0) {
            currentAngle = angles.firstAngle + 360;
        } else {
            currentAngle = angles.firstAngle;
        }
    }

    // TODO this needs better comments, but in the meantime, its *magic*
    public void move() {
        ControlConfig.update(gamepad1, gamepad2);

        // Deals with tilt. Prevents robot from tilting too far. If above certain tilt angle, corrects it
        // and locks driver control.
        Vector3D robotNormalVec = getRobotNormalVector();
        if (isOverMaxTilt(robotNormalVec) && isBelowUnrecoverableTilt(robotNormalVec)) {
            handleTilt(robotNormalVec);
            return;
        }

        double theta = Math.toRadians(currentAngle);

//        telemetry.addData("CurrentAngle", currentAngle);
//        telemetry.addData("Theta", theta);

        // FIXME if possible make it so that it works with any orientation
        forward = ControlConfig.forward;
        right = ControlConfig.right;
        clockwise = ControlConfig.clockwise;

        temp = (forward * Math.cos(theta) - right * Math.sin(theta));
        side = (forward * Math.sin(theta) + right * Math.cos(theta));

        forward = temp;
        right = side;

//        telemetry.addData("right: ", right);
//        telemetry.addData("forward: ", forward);
//        telemetry.addData("temp: ", temp);
//        telemetry.addData("side: ", side);
//        telemetry.addData("clockwise: ", clockwise);

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
//            telemetry.addLine("super fast");

        } else if (ControlConfig.fast) {
            // Prevent robot from moving fast when linear slide is up. Avoids dangerous tilt issues when
            // robot suddenly stops.
            if (robot.linearSlide.getCurrentPosition() < Constants.getLiftEncoderJunctions()[0] + 10) {
                powerMultiplier = Constants.fastMultiplier;
            } else {
                powerMultiplier = Constants.normalMultiplier;
            }
//            telemetry.addLine("fast");
        } else if (ControlConfig.slow) {
            powerMultiplier = Constants.slowMultiplier;
//            telemetry.addLine("slow");
        } else {
            powerMultiplier = Constants.normalMultiplier;
//            telemetry.addLine("normal");
        }

        telemetry.addData("Power:", powerMultiplier);

        telemetry.update();

        robot.frontLeft.setPower(frontLeft * powerMultiplier);
        robot.frontRight.setPower(-frontRight * powerMultiplier);
        robot.rearLeft.setPower(rearLeft * powerMultiplier);
        robot.rearRight.setPower(-rearRight * powerMultiplier);


    }

    private void handleTilt(Vector3D robotNormalVec) {
        //            Log.d("TiltCorr", "CORRECTION");
        double angleDiff = Math.abs(Math.toDegrees(calcTiltAngle(robotNormalVec)) - Constants.maxTiltDegrees);
        Vector2D responseVec = getNormalAxisProjection(robotNormalVec);

//            double correctionFactor = calcLogisticCorrectionFactor(angleDiff);
        double correctionFactor = calcExponentialCorrectionFactor(angleDiff);
        // Correct based on how far off the axis we are
        responseVec = responseVec.normalize();
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

        setMotorPower(responseVec);
    }

    private double calcExponentialCorrectionFactor(double angleDiff) {
        return Constants.expTotalScale * Math.pow(Math.E, Constants.expAngleScale * Math.abs(angleDiff));
    }

    private boolean isBelowUnrecoverableTilt(Vector3D normalVec) {
        double angleToZ = calcTiltAngle(normalVec);

        return Math.abs(Math.toDegrees(angleToZ)) <= Constants.unrecoverableTiltDegrees;
    }

    public void peripheralMove() {
        ControlConfig.update(gamepad1, gamepad2);

        linearSlideMoveWithOverride();
//        linearSlideMove();

////////////////////GRABBER////////////////////////////////////////////////////////

        // A button = open claw, b button = closed claw
        // Prevents weirdness with gamepad2 locking up
        gamepad2.toString();
        if (ControlConfig.openClaw) {
            isClawClosed = false;
            lastAutoGrab.reset();
            robot.rightClaw.setPosition(Constants.rightClawOpen); // Right claw open
            robot.leftClaw.setPosition(Constants.leftClawOpen); // Left claw open
        } else if (ControlConfig.closeClaw) {
            isClawClosed = true;
            robot.rightClaw.setPosition(Constants.rightClawClosed); // Right claw closed
            robot.leftClaw.setPosition(Constants.leftClawClosed); // Left claw closed
        }

        handleClawAutoGrab();

        // TODO work on this please
//        if (ControlConfig.resetIMU && !areInittingIMU) {
//            telemetry.addLine("Reinitting IMU");
//            telemetry.update();
//            initIMU();
//            areInittingIMU = false;
//            gamepad2.rumble(500);
//        }
    }

    private void handleClawAutoGrab() {
        // Prevent autograbbing at end of teleop
        if (elapsedTime.seconds() >= 115) {
            return;
        }

        if (lastAutoGrab.seconds() < Constants.autoGrabCooldownSeconds) {
            return;
        }

        if (ControlConfig.openClaw) {
            return;
        }

        if (isClawClosed) {
            return;
        }

        if (robot.linearSlide.getCurrentPosition() > Constants.getLiftEncoderJunctions()[0] - 40) {
            return;
        }

        ColorSensor colorSensor = robot.colorSensor;
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

        if (distance > Constants.minAutoGrabDistance) {
            return;
        }

        if (currentColor == Color.BLUE ? blue > red : red > blue) {
            robot.rightClaw.setPosition(Constants.rightClawClosed);
            robot.leftClaw.setPosition(Constants.leftClawClosed);
            gamepad2.rumble(250);
            isClawClosed = true;
            lastAutoGrab.reset();
        }
    }

    private void initIMU() {
        areInittingIMU = true;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();
    }

    private void linearSlideMoveWithOverride() {
        DcMotor linearSlide = robot.linearSlide;

        telemetry.update();
//        int linearSlideOffetPos = linearSlide.getCurrentPosition() + Constants.linearSlideZeroOffset;
        // Log linear slide offset for debug reasons
//        Log.d("LinearSlideMovement", String.valueOf(Constants.linearSlideZeroOffset));

        // Complex if statement, but if we want to lift the lide and the linear slide is below the max
        // AND we are not overriding, lift the slide
        if (ControlConfig.liftSlide && linearSlide.getCurrentPosition() < Constants.getLiftEncoderMax() && !ControlConfig.overrideModifier) {
            // If we are not in manual mode (basically running to ground or high or something)
            // reset the linear slide target (where the slide wants to go) to the current position
            if (linearSlideMode != LinearSlideMode.MANUAL) {
                linearSlideTarget = linearSlide.getCurrentPosition();
            }

            // Reset linear slide mode to manual, for obvious reasons
            linearSlideMode = LinearSlideMode.MANUAL;
            // Set the target to the MINIMUM of either the current target plus some number or the
            // linear slide max to prevent linear slide from going too high
            linearSlideTarget = Math.min(linearSlideTarget + Constants.upEncoderStep, Constants.getLiftEncoderMax());

        // Same logic as above, just for going down. Make sure to use zeroOffset and not 0 since
        // we don't know where 0 is and zeroOffset should be the minimum for the slide
        } else if (ControlConfig.lowerSlide && linearSlide.getCurrentPosition() > Constants.linearSlideZeroOffset && !ControlConfig.overrideModifier) {
            if (linearSlideMode != LinearSlideMode.MANUAL) {
                linearSlideTarget = linearSlide.getCurrentPosition();
            }

            linearSlideMode = LinearSlideMode.MANUAL;

            // Same as the min above, just now with max and zeroOffset. Prevents slide from going down
            // too far.
            linearSlideTarget = Math.max(linearSlideTarget - Constants.downEncoderStep, Constants.linearSlideZeroOffset);

        // If we are not overriding and in manual mode, set target to current position. Prevents
        // slide from moving after button is released.
        } else if (linearSlideMode == LinearSlideMode.MANUAL && !ControlConfig.overrideModifier) {
            linearSlideTarget = linearSlide.getCurrentPosition();
        }

        // All these are the same. If we pushed the button to go to a certain location, set the
        // target there and change the mode

        // Ella safety.
        // For the ground, we don't want to go to ground if the claw is closed and we are above
        // the low junction
        long linearSlidePos = robot.linearSlide.getCurrentPosition();
        long ellaSafetyThreshold = Constants.getLiftEncoderJunctions()[0] - 20;
        if (ControlConfig.goToGround && !(isClawClosed && linearSlidePos > ellaSafetyThreshold)) {
            linearSlideMode = LinearSlideMode.GROUD;
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

        // Handle overrides since we ignore them above. Some of the logic is reused and I will only
        // comment new logic

        // If we override and are lifting the slide
        if (ControlConfig.overrideModifier && ControlConfig.liftSlide) {
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
        telemetry.addData("Linear Slide set pos", linearSlideTarget);
//        Log.d("LinearSlide", String.valueOf(linearSlideTarget));
        telemetry.update();
    }

    private void setMotorPower(Vector2D responseVec) {
        // For some reason, as it is now, this causes spinning when correcting, but I am too lazy
        // to fix. It's not a bug, it's a feature, I guess.
        double x = responseVec.getX();
        double y = responseVec.getY();
        double frontLeftPower = y - x;
        double rearLeftPower = y - x;
        double rearRightPower = -y - x;
        double frontRightPower = x + y;
        robot.drive.setMotorPowers(frontLeftPower, rearLeftPower, rearRightPower, frontRightPower);
    }

    private double calcTiltAngle(Vector3D robotNormalVec) {
        return Math.acos(robotNormalVec.getZ());
    }

    // Uses a logarithmic growth sigmoid function to calculate correction
    private double calcLogarithmicCorrectionFactor(double angleDiff) {
        return Constants.tiltCorrectionLogisticScale * Math.log(Constants.tiltCorrectionValueScale * Math.abs(angleDiff) + 1);
    }

    private Vector3D getRobotNormalVector() {
        Orientation orientation = imu.getAngularOrientation();

        // AxesOrder.indices() returns an array of integers corresponding to what order the axes are
        // in. For example, if the AxesOrder is ZYX, indices() is [2, 1, 0], since the X-axis (pos 1
        // of the indices array] is the third reported angle (index 2).
        int[] axisIndicies = orientation.axesOrder.indices();

        // All these calcuations work only in radians, so gotta do this
        orientation = orientation.toAngleUnit(AngleUnit.RADIANS);

        double[] angles = new double[]{orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle};

        // All this code basically just figures out how the IMU is reporting angles and saves that
        // in a way we can acually deal with
        double x = angles[axisIndicies[0]];
        double y = angles[axisIndicies[1]];
        double z = angles[axisIndicies[2]];

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
        //        Vector3D normalVec = new Vector3D(
//                -sin(x) * cos(z) - cos(x) * sin(y) * sin(z),
//                sin(x) * sin(z) - cos(x) * sin(y) * cos(z),
//                cos(x) * cos(y)
//        );
        Vector3D normalVec = new Vector3D(
                sin(x) * cos(y) * cos(z) + sin(y) * sin(z),
                sin(y) * cos(z) - sin(x) * cos(y) * sin(z),
                cos(x) * cos(y)
        );
        // Just in case. Above should be normal, but just in case.
        // Also, can't normalize vectors of length 0 so gotta do this
        try {
            normalVec = normalVec.normalize();
        } catch (MathArithmeticException e) {

        }
//        telemetry.addData("Component x", normalVec.getX());
//        telemetry.addData("Component y", normalVec.getY());
//        telemetry.addData("Component z", normalVec.getZ());

//        Log.d("Tilt", String.format("Component x %f", normalVec.getX()));
//        Log.d("Tilt", String.format("Component y %f", normalVec.getY()));
//        Log.d("Tilt", String.format("Component z %f", normalVec.getZ()));

        telemetry.update();

        return normalVec;
    }

    private boolean isOverMaxTilt(Vector3D normalVec) {
        // To compute the angle to the Z axis (tilt angle) we need the dot product with this vector
        // and the Z axis, but fortunately the dot product simplifies down to this since two of
        // the components of the z unit vector are zero (0, 0, 1)
        double angleToZ = calcTiltAngle(normalVec);

        return Math.abs(Math.toDegrees(angleToZ)) >= Constants.maxTiltDegrees;
    }

    private Vector2D getNormalAxisProjection(Vector3D normalVec) {
        // Generates the ideal robot response to the given normal vector. Does not normalize intentionally
        // as we don't want to respond with full force no matter what
        return new Vector2D(normalVec.getX(), normalVec.getY());
    }

    /*-----------------------------------//
     * DO NOT WRITE CODE BELOW THIS LINE  *
     * -----------------------------------*/
    void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

//        telemetry.addLine()
//                .addData("status", new Func<String>() {
//                    @Override public String value() {
//                        return imu.getSystemStatus().toShortString();
//                    }
//                })
//                .addData("calib", new Func<String>() {
//                    @Override public String value() {
//                        return imu.getCalibrationStatus().toString();
//                    }
//                });
//
//        telemetry.addLine()
//                .addData("heading", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.firstAngle);
//                    }
//                })
//                .addData("roll", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.secondAngle);
//                    }
//                })
//                .addData("pitch", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.thirdAngle);
//                    }
//                });
//
//        telemetry.addLine()
//                .addData("grvty", new Func<String>() {
//                    @Override public String value() {
//                        return gravity.toString();
//                    }
//                })
//                .addData("mag", new Func<String>() {
//                    @Override public String value() {
//                        return String.format(Locale.getDefault(), "%.3f",
//                                Math.sqrt(gravity.xAccel*gravity.xAccel
//                                        + gravity.yAccel*gravity.yAccel
//                                        + gravity.zAccel*gravity.zAccel));
//                    }
//                });

        // telemetry.addData("currentAngle", "%.1f", currentAngle);
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
