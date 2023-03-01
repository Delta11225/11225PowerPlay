package org.firstinspires.ftc.teamcode.testing;

import android.graphics.Canvas;
import android.util.Log;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.stream.Collectors;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

@Disabled
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
//        robot.drive.getLocalizer().setPoseEstimate(Constants.currentPose);
        robot.drive.getLocalizer().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(270)));

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
                Log.d("Targeting", "Target point: " + targetPosition.getX() + " " + targetPosition.getY());

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
            Vector2d robotVec = new Vector2d(robotPos.getX(), robotPos.getY());
            junctionsToCheck.sort((v1, v2) -> (int) (Math.abs(robotVec.angleBetween(v1) - robotVec.angleBetween(v2)) * 10));
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

    private void generateJunctions() {
        double fieldWidth = 70 * 2;
        double offset = fieldWidth / 6.0;

        // Generate the junction positions. The 2.5 is because I am worried about floating point errors.
        for (double i = -offset * 2; i < offset * 2.5; i += offset) {
            for (double j = -offset * 2; j < offset * 2.5; j += offset) {
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
        double fieldWidth = 70 * 2;
        double offset = fieldWidth / 6.0;

        // The maximum possible distance between two adjacent junctions. Used to ignore anything farther away.
        double maxJunctionDist = vectorPointDistance(new Vector2d(0, 0), new Vector2d(offset * 1.1, offset * 1.1));

        // TODO untested. Make sure sorts in correct order.
        // FIXME i guess stream and collect doesn't actually preserve sort order?
        List<Vector2d> sorted_closest = junctionPositions.stream()
                .filter(v1 -> (vectorPointDistance(v1, robotPos) <= maxJunctionDist))
                .collect(Collectors.toList());

        sorted_closest.sort((v1, v2) -> (int) (Math.abs(vectorPointDistance(v1, robotPos) - vectorPointDistance(v2, robotPos) * 10)));

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
