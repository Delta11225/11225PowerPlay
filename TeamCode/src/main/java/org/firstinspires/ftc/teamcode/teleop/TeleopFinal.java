package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.ControlConfig;
import org.firstinspires.ftc.teamcode.util.Hardware23;
import org.firstinspires.ftc.teamcode.util.LinearSlideMode;

import java.util.Locale;

//@Disabled
@TeleOp
public class TeleopFinal extends OpMode {
    // FIXME in the future reduce number of global vars
    // TODO add button combination to override things
    // TODO like maybe right joystick button and a, b, x, or y

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

        composeTelemetry();
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

        // FIXME Hardware23 takes forever to init for some reason
        telemetry.addData("Robot HWMap init time", getRuntime());
        telemetry.update();
    }

    @Override
    public void start() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        currentAngle = 0;

        elapsedTime.reset();
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

    public void move() {

        ControlConfig.update(gamepad1, gamepad2);
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
            powerMultiplier = Constants.fastMultiplier;
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

    public void peripheralMove() {
        ControlConfig.update(gamepad1, gamepad2);

        linearSlideMoveWithOverride();
//        linearSlideMove();

////////////////////GRABBER////////////////////////////////////////////////////////

        // A button = open claw, b button = closed claw
        gamepad2.toString();
        if (ControlConfig.openClaw) {
            robot.rightClaw.setPosition(Constants.rightClawOpen); // Right claw open
            robot.leftClaw.setPosition(Constants.leftClawOpen); // Left claw open
        } else if (ControlConfig.closeClaw) {
            robot.rightClaw.setPosition(Constants.rightClawClosed); // Right claw closed
            robot.leftClaw.setPosition(Constants.leftClawClosed); // Left claw closed
        }

    }

    private void linearSlideMoveWithOverride() {
        // FIXME linear slide moves extremely slowly after running to ground for some reason
        DcMotor linearSlide = robot.linearSlide;
//        int linearSlideOffetPos = linearSlide.getCurrentPosition() + Constants.linearSlideZeroOffset;
        Log.d("LinearSlideMovement", String.valueOf(Constants.linearSlideZeroOffset));
        if (ControlConfig.liftSlide && linearSlide.getCurrentPosition() < Constants.getLiftEncoderMax() && !ControlConfig.overrideModifier) {
            if (linearSlideMode != LinearSlideMode.MANUAL) {
                linearSlideTarget = linearSlide.getCurrentPosition();
            }

            linearSlideMode = LinearSlideMode.MANUAL;
            linearSlideTarget = Math.min(linearSlideTarget + Constants.upEncoderStep, Constants.getLiftEncoderMax());
        } else if (ControlConfig.lowerSlide && linearSlide.getCurrentPosition() > Constants.linearSlideZeroOffset && !ControlConfig.overrideModifier) {
            if (linearSlideMode != LinearSlideMode.MANUAL) {
                linearSlideTarget = linearSlide.getCurrentPosition();
            }

            linearSlideMode = LinearSlideMode.MANUAL;
            linearSlideTarget = Math.max(linearSlideTarget - Constants.downEncoderStep, Constants.linearSlideZeroOffset);
        } else if (linearSlideMode == LinearSlideMode.MANUAL && !ControlConfig.overrideModifier) {
            linearSlideTarget = linearSlide.getCurrentPosition();
        }

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
            telemetry.addLine("Can't do this yet, moron. Add the linear slide first.");
            telemetry.update();
            linearSlideTarget = Constants.getLiftEncoderJunctions()[0];
//            linearSlideTarget = Constants.getLiftEncoderJunctions()[1];
        }

        if (ControlConfig.goToHigh) {
            linearSlideMode = LinearSlideMode.HIGH;
            telemetry.addLine("Can't do this yet, moron. Add the linear slide first.");
            telemetry.update();
            linearSlideTarget = Constants.getLiftEncoderJunctions()[0];
//            linearSlideTarget = Constants.getLiftEncoderJunctions()[2];
        }

        // Handle overrides
        if (ControlConfig.overrideModifier && ControlConfig.liftSlide) {
            if (linearSlideMode != LinearSlideMode.MANUAL) {
                linearSlideTarget = linearSlide.getCurrentPosition();
            }

            linearSlideTarget = linearSlideTarget + Constants.upEncoderStep;
            Constants.linearSlideZeroOffset = linearSlide.getCurrentPosition();
        }

        if (ControlConfig.overrideModifier && ControlConfig.lowerSlide) {
            if (linearSlideMode != LinearSlideMode.MANUAL) {
                linearSlideTarget = linearSlide.getCurrentPosition();
            }

            linearSlideTarget = linearSlideTarget - Constants.upEncoderStep;
            Constants.linearSlideZeroOffset = linearSlide.getCurrentPosition();
        }

        robot.linearSlide.setTargetPosition(linearSlideTarget);
        robot.linearSlide.setPower(0.5);
        telemetry.addData("Linear Slide set pos", linearSlideTarget);
        telemetry.update();
    }

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
        robot.linearSlide.setPower(0.5);
        telemetry.addData("Linear Slide set pos", linearSlideTarget);
        telemetry.update();
    }

    private int calcWithOffset(int rawLinearSlideTarget) {
        return rawLinearSlideTarget + Constants.linearSlideZeroOffset;
    }

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

//    class LiftThread implements Runnable {
//        @Override
//        public void run() {
//            while (opModeIsActive()) {
//                if (ControlConfig.runSlideToLowDump) {
//                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////                        telemetry.addLine("Got here");
//                    telemetry.update();
//                    robot.liftMotor.setTargetPosition((int) (liftEncoderStart + 1100));
//                    robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                    robot.liftMotor.setPower(1.0);
//                    slideReturning = true;
////                        telemetry.addLine(String.valueOf(slideReturning));
//                    telemetry.update();
//                }
//
//                while (opModeIsActive() && robot.liftMotor.isBusy()) {}
//                slideReturning = false;
//            }
//        }
//    }
}
