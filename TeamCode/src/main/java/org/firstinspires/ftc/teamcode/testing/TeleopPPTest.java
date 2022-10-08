package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.ftccommon.SoundPlayer;
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
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.ControlConfig;
import org.firstinspires.ftc.teamcode.util.Hardware22;
import org.firstinspires.ftc.teamcode.util.Hardware23;

import java.util.Locale;

//@Disabled
@TeleOp
public class TeleopPPTest extends LinearOpMode {
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

    boolean tseArmActive = false;
    boolean tseRodActive = false;
    boolean unpressed = true;
    boolean liftmode = true;

    volatile boolean slideReturning = false;

    double tsePos = Constants.tseArmInitPosition;
    double liftEncoderStart;

    double duckWheelSpeed = 0;
    double duckWheelMaxSpeed = Constants.towerWheelSpeedEndgame;

    boolean motivated = false;
    boolean hasRumbled = false;
    boolean didRumble1 = false;

    double offset = 0;
    int holdPosition;
    ElapsedTime elapsedTime = new ElapsedTime();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

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

        // End init phase
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        currentAngle = 0;

        elapsedTime.reset();
        while (opModeIsActive()) {
            ControlConfig.update(gamepad1, gamepad2);

            telemetry.update();
            move();
//            peripheralMove();
//            handleMotivation();

            if (angles.firstAngle < 0) {
                currentAngle = angles.firstAngle + 360;
            } else {
                currentAngle = angles.firstAngle;
            }
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

        /////////////////////////////LINEAR SLIDE//////////////////////////////
        if (gamepad1.dpad_up && robot.linearSlide.getCurrentPosition() < 1555) {
            robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.linearSlide.setPower(0.5);
            if (robot.linearSlide.getCurrentPosition() > 1555) {
                holdPosition = 1555;
            } else {
                holdPosition = robot.linearSlide.getCurrentPosition();
            }
        } else if (gamepad1.dpad_down && robot.linearSlide.getCurrentPosition() > 0) {
            robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.linearSlide.setPower(-0.4);
            holdPosition = robot.linearSlide.getCurrentPosition();
        } else {
            if (robot.linearSlide.getCurrentPosition() < 1555 && robot.linearSlide.getCurrentPosition() > 600) {
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

////////////////////GRABBER////////////////////////////////////////////////////////

        // A button = open claw, b button = closed claw
        if (gamepad1.a) {
            robot.rightClaw.setPosition(0.95); // Right claw open
            robot.leftClaw.setPosition(0.0); // Left claw open
        }
        if (gamepad1.b) {
            robot.rightClaw.setPosition(0.70); // Right claw closed
            robot.leftClaw.setPosition(0.25); // Left claw closed
        }

    }

    public void handleMotivation() {
        if (ControlConfig.playMotivSound && !motivated) {
            int motivNum = (int) (Math.random() * (Constants.motivationQuantity));
            int motivID = hardwareMap.appContext.getResources().getIdentifier("motivate_" + motivNum, "raw", hardwareMap.appContext.getPackageName());

            boolean motivFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, motivID);
            if (motivFound) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, motivID);
            }
            motivated = true;
        } else if (!ControlConfig.playMotivSound) {
            motivated = false;
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
