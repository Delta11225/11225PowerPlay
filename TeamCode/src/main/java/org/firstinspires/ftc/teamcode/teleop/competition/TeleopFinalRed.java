package org.firstinspires.ftc.teamcode.teleop.competition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware22;

import java.util.Locale;

@TeleOp(name="TeleOp final red")
//@Disabled
// TODO consider adding button to ignore all safeties
public class TeleopFinalRed extends LinearOpMode {

    Hardware22 robot;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    private ElapsedTime runtime = new ElapsedTime();

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

    boolean tseArmActive = false;
    boolean tseRodActive = false;
    boolean unpressed = true;

    double tsePos = Constants.tseArmInitPosition;
    double liftEncoderStart;

    double duckWheelSpeed = 0;
    double duckWheelMaxSpeed = -Constants.towerWheelSpeedEndgame;

    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        composeTelemetry();

        robot = new Hardware22(hardwareMap);

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.rearLeft.setPower(0);
        robot.rearRight.setPower(0);

        robot.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.rearLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.rearRight.setDirection(DcMotor.Direction.REVERSE);

        robot.dumpServo.setPosition(Constants.collectPosition);

        robot.tseServo.setPosition(Constants.tseArmInitPosition);

        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftEncoderStart = robot.liftMotor.getCurrentPosition();

        // End init phase
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        currentAngle = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            ControlConfig.update(gamepad1, gamepad2);

            telemetry.update();

            while (angles.firstAngle < 0 && opModeIsActive()) {
                telemetry.update();
                move();
                peripheralMove();

                currentAngle = angles.firstAngle + 360;
//                telemetry.addData("currentAngle loop 1", "%.1f", currentAngle);
            }

            while (angles.firstAngle >= 0 && opModeIsActive()) {


                telemetry.update();
                move();
                peripheralMove();

                currentAngle = angles.firstAngle;
//                telemetry.addData("currentAngle loop 2", "%.1f", currentAngle);
            }

//            telemetry.addLine("null angle");

        }
    }


    public void move(){
        ControlConfig.update(gamepad1, gamepad2);
        double theta = Math.toRadians(currentAngle);

//        telemetry.addData("CurrentAngle", currentAngle);
//        telemetry.addData("Theta", theta);

        forward = ControlConfig.right;
        right = ControlConfig.backward;
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
            powerMultiplier = Constants.superfastMultiplier;
//            telemetry.addLine("super fast");
        } else if (ControlConfig.fast){
            powerMultiplier = Constants.fastMultiplier;
//            telemetry.addLine("fast");
        } else if (ControlConfig.slow) {
            powerMultiplier = Constants.slowMultiplier;
//            telemetry.addLine("slow");
        }  else {
            powerMultiplier = Constants.normalMultiplier;
//            telemetry.addLine("normal");
        }

        telemetry.update();


        robot.frontLeft.setPower(frontLeft * powerMultiplier);
        robot.frontRight.setPower(-frontRight * powerMultiplier);
        robot.rearLeft.setPower(rearLeft * powerMultiplier);
        robot.rearRight.setPower(-rearRight * powerMultiplier);


    }

    public void peripheralMove(){
        ControlConfig.update(gamepad1, gamepad2);
        telemetry.addData("Encoder count, min 1100 to dump", robot.liftMotor.getCurrentPosition() - liftEncoderStart);
        telemetry.update();

        // Dumping bucket servo
        if (ControlConfig.dumpBucket && robot.liftMotor.getCurrentPosition() > liftEncoderStart + 1100) {
            robot.dumpServo.setPosition(Constants.dumpPosition);
        } else if (ControlConfig.collectBucket) {
            robot.dumpServo.setPosition(Constants.collectPosition);
        }

        // Tower motor
        if (ControlConfig.duckWheelRed) {
            robot.towerMotor.setPower(duckWheelSpeed);

            duckWheelSpeed += .5;
            if (duckWheelSpeed > duckWheelMaxSpeed) {
                duckWheelSpeed = duckWheelMaxSpeed;
            }
        } else {
            robot.towerMotor.setPower(0);
            duckWheelSpeed = 0;
        }

        // Set dumpServo to hold position if above certain encoder count
        // For now, this has been deemed unnecessary
        if (ControlConfig.liftSlide) { // && robot.liftMotor.getCurrentPosition() > liftEncoderStart + 200) {
            robot.dumpServo.setPosition(Constants.holdPosition);
        }

        // Lift motor
        // Prevent liftMotor from going below start position or going above top position
        // as that would throw off encoders or loosen slack on cable
        if (ControlConfig.liftSlide && robot.liftMotor.getCurrentPosition() < liftEncoderStart + Constants.highEncoder) {
            //added safety here to hold bucket in horizontal position when lifting so freight does not fall out as robot drives around field
            robot.liftMotor.setPower(ControlConfig.linSlideSlow ? Constants.slowMultiplier : 1.0);
        }
        //////SAFETY IN CASE LINEAR SLIDE STARTS UP!!!!
        else if (ControlConfig.lowerSlide && ControlConfig.linearSlideOverride) {
            robot.dumpServo.setPosition(Constants.collectPosition);
            // sleep(300);
            robot.liftMotor.setPower(ControlConfig.linSlideSlow ? -Constants.slowMultiplier : -1.0);
            //reset liftEncoderStart to current postition!!!
            liftEncoderStart = robot.liftMotor.getCurrentPosition();
        } else if (ControlConfig.lowerSlide && robot.liftMotor.getCurrentPosition() > liftEncoderStart + 10) {
            robot.dumpServo.setPosition(Constants.collectPosition);
            // sleep(300);
            robot.liftMotor.setPower(ControlConfig.linSlideSlow ? -Constants.slowMultiplier : -1.0);
        } else {
            robot.liftMotor.setPower(0);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Collection motor
        // Prevent collect wheel from working if bucket is not down
        if (ControlConfig.collectWheel && robot.liftMotor.getCurrentPosition() <= liftEncoderStart + 100) {
            robot.collectionMotor.setPower(1.0);
        } else if (ControlConfig.unCollectWheel) {
            robot.collectionMotor.setPower(-1.0);
        } else {
            robot.collectionMotor.setPower(0);
        }

        // TSE Arm
        if (ControlConfig.raiseTseArm) {
            tsePos -= Constants.tseStep;
        } else if (ControlConfig.lowerTseArm) {
            tsePos += Constants.tseStep;
            // Not anymore, we got dem hi tork servos
            // Necessary, as otherwise the servos don't have enough torque
//            sleep(500);
        } else if (ControlConfig.initTseArm) {
            tsePos = Constants.tseArmInitPosition;
        } else if (ControlConfig.collectTseArm) {
            tsePos = Constants.tseArmCollectPosition;
        }
        robot.tseServo.setPosition(tsePos);
    }


    /*-----------------------------------//
     * DO NOT WRITE CODE BELOW THIS LINE  *
     * -----------------------------------*/
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
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

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}