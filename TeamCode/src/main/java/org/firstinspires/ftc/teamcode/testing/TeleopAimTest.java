package org.firstinspires.ftc.teamcode.testing;

import android.graphics.Canvas;
import android.util.Log;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.apache.commons.math3.exception.MathArithmeticException;
import org.apache.commons.math3.geometry.Point;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.ControlConfig;
import org.firstinspires.ftc.teamcode.util.Hardware23;
import org.firstinspires.ftc.teamcode.util.LinearSlideMode;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.stream.Collectors;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

//@Disabled
@TeleOp
public class TeleopAimTest extends OpMode {
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
    private List<Vector2d> junctionPositions = new ArrayList<>();

    private Mode currentMode = Mode.NORMAL_CONTROL;
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    // The junction we are aiming at
    private Vector2d targetPosition = new Vector2d(0, 0);

    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }

    @Override
    public void init() {
        resetRuntime();

        generateJunctions();
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Constants.linearSlideZeroOffset = 0;

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

        // Apparently we don't need it
        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drive.getLocalizer().setPoseEstimate(Constants.currentPose);

        // Set input bounds for the heading controller
        // Automatically handles overflow
        headingController.setInputBounds(-Math.PI, Math.PI);

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
        robot.drive.getLocalizer().update();

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
        if (isOverMaxTilt(robotNormalVec)) {
            handleTiltCorrection(robotNormalVec);
            return;
        }

        switch (currentMode) {
            case NORMAL_CONTROL:
                // TODO add this to control config or something. Make it a toggle?
                // Switch into alignment mode if `a` is pressed
                if (gamepad1.a) {
                    currentMode = Mode.ALIGN_TO_POINT;
                }

                handleStandardMovement();
                break;
            case ALIGN_TO_POINT:
                // Switch back into normal driver control mode if `b` is pressed
                if (gamepad1.b) {
                    currentMode = Mode.NORMAL_CONTROL;
                }

                // Declare a drive direction
                // Pose representing desired x, y, and angular velocity
                Pose2d driveDirection;

                Pose2d poseEstimate = robot.drive.getLocalizer().getPoseEstimate();

                updateTargetPoint(poseEstimate);

                telemetry.addData("mode", currentMode);

                // Create a vector from the gamepad x/y inputs which is the field relative movement
                // Then, rotate that vector by the inverse of that heading for field centric control
                Vector2d fieldFrameInput = new Vector2d(
                        ControlConfig.forward,
                        ControlConfig.left
                );
                Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                // Difference between the target vector and the bot's position
                Vector2d difference = targetPosition.minus(poseEstimate.vec());
                // Obtain the target angle for feedback and derivative for feedforward
                double theta = difference.angle();

                // Not technically omega because its power. This is the derivative of atan2
                double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                // Set the target heading for the heading controller to our desired angle
                headingController.setTargetPosition(theta);

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                double headingInput = (headingController.update(poseEstimate.getHeading())
                        * DriveConstants.kV + thetaFF)
                        * DriveConstants.TRACK_WIDTH;

                // Combine the field centric x/y velocity with our derived angular velocity
                driveDirection = new Pose2d(
                        robotFrameInput,
                        headingInput
                );

                // Update the heading controller with our current heading
                robot.drive.setWeightedDrivePower(driveDirection);
                headingController.update(poseEstimate.getHeading());
                break;
        }
}

    private void updateTargetPoint(Pose2d robotPos) {
        List<Vector2d> closest = getClosestJunctions(robotPos);
        List<Vector2d> junctionsToCheck = filterJunctionsByQuadrant(findFacingQuadrant(), closest, robotPos);

        if (junctionsToCheck.size() == 0) {
            junctionsToCheck = closest;
        } else {
            junctionsToCheck = junctionsToCheck.stream()
                    .sorted((v1, v2) -> (int) ((v1.angleBetween(v2) * 10)))
                    .collect(Collectors.toList());
        }

        targetPosition = junctionsToCheck.get(0);
    }

    private void handleStandardMovement() {
        double theta = Math.toRadians(currentAngle);

//        telemetry.addData("CurrentAngle", currentAngle);
//        telemetry.addData("Theta", theta);

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

    private void handleTiltCorrection(Vector3D robotNormalVec) {
        Log.d("TiltCorr", "CORRECTION");
        double angleDiff = Constants.maxTiltDegrees - Math.toDegrees(calcTiltAngle(robotNormalVec));
        Vector2D responseVec = getNormalAxisProjection(robotNormalVec);

        // Correct based on how far off the axis we are
        responseVec = responseVec.normalize();
        responseVec = responseVec.scalarMultiply(calcCorrectionFactor(angleDiff));

        Log.d("TiltCorr", String.format("Angle diff %f", (angleDiff)));
        Log.d("TiltCorr", String.format("Correction factor %f", calcCorrectionFactor(angleDiff)));

        double responseX = responseVec.getX();
        double responseY = responseVec.getY();
        telemetry.addData("Response vec x", responseX);
        telemetry.addData("Response vec y", responseY);
        telemetry.update();

        Log.d("TiltCorr", String.format("Response vec x %f", responseX));
        Log.d("TiltCorr", String.format("Response vec y %f", responseY));

        setMotorPower(responseVec);
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
            robot.rightClaw.setPosition(Constants.rightClawOpen); // Right claw open
            robot.leftClaw.setPosition(Constants.leftClawOpen); // Left claw open
        } else if (ControlConfig.closeClaw) {
            robot.rightClaw.setPosition(Constants.rightClawClosed); // Right claw closed
            robot.leftClaw.setPosition(Constants.leftClawClosed); // Left claw closed
        }

        // TODO work on this please
//        if (ControlConfig.resetIMU && !areInittingIMU) {
//            telemetry.addLine("Reinitting IMU");
//            telemetry.update();
//            initIMU();
//            areInittingIMU = false;
//            gamepad2.rumble(500);
//        }
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
        // TODO can we somehow track motor power and stop linear slide if it is drawing too much current?
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
        if (ControlConfig.goToGround) {
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
                linearSlideTarget = linearSlide.getCurrentPosition();
            }

            linearSlideTarget = linearSlideTarget + Constants.upEncoderStep;

            // Reset the zeroOffset to the current position, as that is supposedly the new 0.
            Constants.linearSlideZeroOffset = linearSlide.getCurrentPosition();
        }

        // Same as above.
        if (ControlConfig.overrideModifier && ControlConfig.lowerSlide) {
            if (linearSlideMode != LinearSlideMode.MANUAL) {
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
        Log.d("LinearSlide", String.valueOf(linearSlideTarget));
        telemetry.update();
    }

    @Deprecated
    private void linearSlideMove() {
        DcMotor linearSlide = robot.linearSlide;
        // Use this modified variable for any bounds checking. For target setting it will be offset
        // at the end of the method, so just use linearSlide.getCurrentPosition() for that
        int linearSlideComparisonPos = linearSlide.getCurrentPosition() - Constants.linearSlideZeroOffset;

        if (ControlConfig.liftSlide && ((linearSlideComparisonPos < Constants.getLiftEncoderMax()) || ControlConfig.overrideModifier)) {
            if (linearSlideMode != LinearSlideMode.MANUAL) {
                linearSlideTarget = linearSlide.getCurrentPosition();
            }

            linearSlideMode = LinearSlideMode.MANUAL;
            if (ControlConfig.overrideModifier) {
                linearSlideTarget = linearSlideTarget + Constants.upEncoderStep;
                Constants.linearSlideZeroOffset = robot.linearSlide.getCurrentPosition();
            } else {
                linearSlideTarget = Math.min(linearSlideTarget + Constants.upEncoderStep, Constants.getLiftEncoderMax());
            }
        } else if (ControlConfig.lowerSlide && ((linearSlideComparisonPos > 0) || ControlConfig.overrideModifier)) {
            if (linearSlideMode != LinearSlideMode.MANUAL) {
                linearSlideTarget = linearSlide.getCurrentPosition();
            }

            linearSlideMode = LinearSlideMode.MANUAL;
            if (ControlConfig.overrideModifier) {
                linearSlideTarget = linearSlideTarget - Constants.downEncoderStep;
                Constants.linearSlideZeroOffset = robot.linearSlide.getCurrentPosition();
            } else {
                linearSlideTarget = Math.max(linearSlideTarget - Constants.downEncoderStep, 0);
            }
        } else if (linearSlideMode == LinearSlideMode.MANUAL) {
            linearSlideTarget = linearSlide.getCurrentPosition();
        }

        if (ControlConfig.goToLow) {
            linearSlideMode = LinearSlideMode.LOW;
            linearSlideTarget = Constants.getLiftEncoderJunctions()[0];
        }

        if (ControlConfig.goToGround) {
            linearSlideMode = LinearSlideMode.GROUD;
            linearSlideTarget = Constants.linearSlideZeroOffset;
        }

        robot.linearSlide.setTargetPosition(calcWithOffset(linearSlideTarget));
        robot.linearSlide.setPower(Constants.liftPosRunPower);
        telemetry.addData("Linear Slide set pos", linearSlideTarget);
        telemetry.update();
    }

    @Deprecated
    private int calcWithOffset(int rawLinearSlideTarget) {
        return rawLinearSlideTarget + Constants.linearSlideZeroOffset;
    }

    @Deprecated
    private void linearSlideMoveOld() {
        /////////////////////////////LINEAR SLIDE//////////////////////////////
        if (ControlConfig.liftSlide && robot.linearSlide.getCurrentPosition() < Constants.getLiftEncoderMax()) {
            // If we give it any input, stop running to a certain set position
//            runningToPos = false;
            linearSlideMode = LinearSlideMode.MANUAL;
            robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.linearSlide.setPower(Constants.liftUpPower);
            if (robot.linearSlide.getCurrentPosition() > Constants.getLiftEncoderMax()) {
                holdPosition = Constants.getLiftEncoderMax();
            } else {
                holdPosition = robot.linearSlide.getCurrentPosition();
            }
        } else if (ControlConfig.lowerSlide && robot.linearSlide.getCurrentPosition() > 0) {
//            runningToPos = false;
            linearSlideMode = LinearSlideMode.MANUAL;
            robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.linearSlide.setPower(-Constants.liftDownPower);
            holdPosition = robot.linearSlide.getCurrentPosition();
        } else if (linearSlideMode == LinearSlideMode.MANUAL) {
            if (robot.linearSlide.getCurrentPosition() < Constants.getLiftEncoderMax() && robot.linearSlide.getCurrentPosition() > 600) {
                robot.linearSlide.setTargetPosition(holdPosition);
                robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.linearSlide.setPower(0.05);
                //linearSlide.setPower(0);
            } else {
                robot.linearSlide.setPower(0);
                robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            telemetry.addData("encoder", robot.linearSlide.getCurrentPosition());
            telemetry.update();
        }

        // Handle rumblies for low junction drop pos
        int currentPos = robot.linearSlide.getCurrentPosition();
        // liftEncoderLow is the lowest dump pos, but we add some encoder counts to it to make a larger
        // rumble zone as robot could just skip past it
        if (currentPos > Constants.getLiftEncoderJunctions()[0] && currentPos < Constants.getLiftEncoderJunctions()[0] + 60) {
            // There is probably a better way to do this, but basically, rumble if we haven't
            // rumbled while we have been in the rumble range. Reset when we leave it
            if (!hasRumbled) {
                gamepad2.rumble(250);
            }
            hasRumbled = true;
        } else {
            hasRumbled = false;
        }

        // Initialization code. If we want to start running goToLow, set appropriate target pos,
        // put linear slide in correct mode, and tell everyone we are running to a position.
        if (ControlConfig.goToLow && linearSlideMode != LinearSlideMode.LOW) {
            telemetry.addData("Go to pos", Constants.getLiftEncoderJunctions()[0]);
            telemetry.update();

            robot.linearSlide.setTargetPosition(Constants.getLiftEncoderJunctions()[0]);
            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            runningToPos = true;
            linearSlideMode = LinearSlideMode.LOW;
        }

        // Same as above, but running to bottom
        if (ControlConfig.goToGround && linearSlideMode != LinearSlideMode.GROUD) {
            telemetry.addData("Go to pos", 0);
            telemetry.update();

            robot.linearSlide.setTargetPosition(0);
            robot.linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            runningToPos = true;
            linearSlideMode = LinearSlideMode.GROUD;
        }

        // If we are running to a position and the slide is busy, set its power to .5. Otherwise,
        // set its power to 0 and tell everyone we are done running to a position
        if (linearSlideMode != LinearSlideMode.MANUAL) {
            if (robot.linearSlide.isBusy()) {
                robot.linearSlide.setPower(Constants.liftPosRunPower);
            } // else {
//                runningToPos = false;
//                robot.linearSlide.setPower(0);
//            }
        }
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
    private double calcCorrectionFactor(double angleDiff) {
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

        telemetry.addData("Angle 1 (x)", x);
        telemetry.addData("Angle 2 (y)", y);
        telemetry.addData("Angle 3 (z)", z);

        Log.d("Tilt", String.format("Angle 1 (x) %f", x));
        Log.d("Tilt", String.format("Angle 2 (y) %f", y));
        Log.d("Tilt", String.format("Angle 3 (z) %f", z));

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
        telemetry.addData("Component x", normalVec.getX());
        telemetry.addData("Component y", normalVec.getY());
        telemetry.addData("Component z", normalVec.getZ());

        Log.d("Tilt", String.format("Component x %f", normalVec.getX()));
        Log.d("Tilt", String.format("Component y %f", normalVec.getY()));
        Log.d("Tilt", String.format("Component z %f", normalVec.getZ()));

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

    private void generateJunctions() {
        double fieldWidth = 70;
        double offset = fieldWidth / 6;

        // Generate the junction positions. The 2.5 is because I am worried about floating point errors.
        for (double i = -offset * 2; i < offset * 2.5; i += offset) {
            for (double j = -offset * 2; i < offset * 2.5; i += offset) {
                junctionPositions.add(new Vector2d(i, j));
            }
        }
    }

    /**
     * Returns the distance between two vectors when treating their components like coordinates.
     * @param v1 The first vector
     * @param v2 The second vector
     * @return The distance between them, with the components treated like ordered pairs.
     */
    private double vectorPointDistance(Vector2d v1, Vector2d v2) {
        double ac = Math.abs(v2.getY() - v1.getY());
        double cb = Math.abs(v2.getX() - v1.getX());

        return Math.hypot(ac, cb);

    }

    private double vectorPointDistance(Vector2d v1, Pose2d v2) {
        double ac = Math.abs(v2.getY() - v1.getY());
        double cb = Math.abs(v2.getX() - v1.getX());

        return Math.hypot(ac, cb);

    }

    private List<Vector2d> getClosestJunctions(Pose2d robotPos) {
        // The maximum possible distance between two adjacent junctions. Used to ignore anything farther away.
        double maxJunctionDist = vectorPointDistance(new Vector2d(0, 0), new Vector2d(offset * 1.1, offset * 1.1));

        // TODO untested. Make sure sorts in correct order.
        List<Vector2d> sorted_closest = junctionPositions.stream()
                .filter(v1 -> (vectorPointDistance(v1, robotPos) < maxJunctionDist))
                .sorted((v1, v2) -> (int) ((vectorPointDistance(v1, v2) * 10)))
                .collect(Collectors.toList());

        return sorted_closest;
    }

    private int findFacingQuadrant() {
        Orientation orientation = imu.getAngularOrientation();
        int[] axisIndicies = orientation.axesOrder.indices();
        orientation = orientation.toAngleUnit(AngleUnit.DEGREES);
        double[] angles = new double[]{orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle};

        // We only care about where the robot is facing
        double z = angles[axisIndicies[2]];

        // Mod function actually deals with negatives for us, yay!
        double rotation = z % 360;

        // Figure out if robot is in first, second, third, or 4th quadrant
        if (0 <= rotation && rotation < 90) {
            return 1;
        }
        else if (90 <= rotation && rotation < 180) {
            return 2;
        }
        else if (180 <= rotation && rotation < 270) {
            return 3;
        }
        return 4;
    }
    
    private boolean isJunctionInQuadrant(int quadrant, Vector2d junction, Pose2d robotPos) {
        double jx = junction.getX();
        double jy = junction.getY();

        double rx = robotPos.getX();
        double ry = robotPos.getY();
        
        if (quadrant == 1 && jx >= rx && jy <= ry) {
            return true;
        }
        
        if (quadrant == 2 && jx <= rx && jy <= ry) {
            return true;
        }

        if (quadrant == 3 && jx <= rx && jy >= ry) {
            return true;
        }
        
        if (jx >= rx && jy >= ry) {
            return true;
        }

        return false;
    }

    private List<Vector2d> filterJunctionsByQuadrant(int quadrant, List<Vector2d> junctions, Pose2d robotPos) {
        return junctions.stream()
                .filter(j -> isJunctionInQuadrant(quadrant, j, robotPos))
                .collect(Collectors.toList());
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
