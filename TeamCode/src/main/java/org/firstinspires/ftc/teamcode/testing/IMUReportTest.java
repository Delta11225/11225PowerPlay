package org.firstinspires.ftc.teamcode.testing;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.exception.MathArithmeticException;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware23;
import org.firstinspires.ftc.teamcode.util.LinearSlideMode;

@TeleOp
public class IMUReportTest extends OpMode {

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

    @Override
    public void init() {
        resetRuntime();

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Constants.linearSlideZeroOffset = 0;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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
        Vector3D vec = getRobotNormalVector();
        boolean isTilt = isOverMaxTilt(vec);
        telemetry.addData("Is over tilt?", isTilt);
        Log.d("Tilt", String.valueOf(isTilt));
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
                sin(y) * cos(z) - sin(x) * cos(y) * sin(z),
                sin(x) * cos(y) * cos(z) + sin(y) * sin(z),
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
        double angleToZ = Math.acos(normalVec.getZ());

        return Math.abs(Math.toDegrees(angleToZ)) >= Constants.maxTiltDegrees;
    }
}
