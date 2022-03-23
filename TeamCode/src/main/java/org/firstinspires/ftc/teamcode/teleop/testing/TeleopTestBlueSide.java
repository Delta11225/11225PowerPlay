/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware22;
import org.firstinspires.ftc.teamcode.teleop.competition.ControlConfig;

import java.util.Locale;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp test blue side")
//@Disabled
public class TeleopTestBlueSide extends LinearOpMode {

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
        robot.dumpServo.setPosition(0);



        robot.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.rearLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.rearRight.setDirection(DcMotor.Direction.REVERSE);

        robot.dumpServo.setPosition(Constants.collectPosition);

        // End init phase
        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        currentAngle = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            ControlConfig.update(gamepad1, gamepad2);

            telemetry.update();

            while (angles.firstAngle < 0 && opModeIsActive()) {
                ControlConfig.update(gamepad1, gamepad2);

                telemetry.update();
                move();
                peripheralMove();

                currentAngle = angles.firstAngle + 360;
                telemetry.addData("currentAngle loop 1", "%.1f", currentAngle);
            }

            while (angles.firstAngle >= 0 && opModeIsActive()) {
                ControlConfig.update(gamepad1, gamepad2);

                telemetry.update();
                move();
                peripheralMove();

                currentAngle = angles.firstAngle;
                telemetry.addData("currentAngle loop 2", "%.1f", currentAngle);
            }

            telemetry.addLine("null angle");
        }
    }


    public void move(){
        ControlConfig.update(gamepad1, gamepad2);
        double theta = Math.toRadians(currentAngle);

        telemetry.addData("CurrentAngle", currentAngle);
        telemetry.addData("Theta", theta);

        forward = ControlConfig.left;
        right = ControlConfig.forward;
        clockwise = ControlConfig.clockwise;

        temp = (forward * Math.cos(theta) - right * Math.sin(theta));
        side = (forward * Math.sin(theta) + right * Math.cos(theta));

        forward = temp;
        right = side;

        telemetry.addData("right: ", right);
        telemetry.addData("forward: ", forward);
        telemetry.addData("temp: ", temp);
        telemetry.addData("side: ", side);
        telemetry.addData("clockwise: ", clockwise);

        frontLeft = forward + right + clockwise;
        rearLeft = forward - right + clockwise;
        rearRight = forward + right - clockwise;
        frontRight = forward - right - clockwise;

        telemetry.addData("front left: ", frontLeft);
        telemetry.addData("rear left: ", rearLeft);
        telemetry.addData("rear right: ", rearRight);
        telemetry.addData("front right: ", frontRight);

        // Handle speed control
        if (ControlConfig.fast){
            powerMultiplier = Constants.fastMultiplier;
            telemetry.addLine("fast");
        } else if (ControlConfig.slow) {
            powerMultiplier = Constants.slowMultiplier;
            telemetry.addLine("slow");
        } else {
            powerMultiplier = Constants.normalMultiplier;
            telemetry.addLine("normal");
        }

        telemetry.update();


        robot.frontLeft.setPower(frontLeft * powerMultiplier);
        robot.frontRight.setPower(-frontRight * powerMultiplier);
        robot.rearLeft.setPower(rearLeft * powerMultiplier);
        robot.rearRight.setPower(-rearRight * powerMultiplier);


    }

    public void peripheralMove(){
        ControlConfig.update(gamepad1, gamepad2);
        // Dumping servo
        if (ControlConfig.dumpBucket) {
            robot.dumpServo.setPosition(Constants.dumpPosition);
        } else if (ControlConfig.collectBucket) {
            robot.dumpServo.setPosition(Constants.collectPosition);
        }

        // Tower motor
        if (ControlConfig.duckWheelBlue) {
            robot.towerMotor.setPower(Constants.towerWheelSpeedEndgame);
        } else if (ControlConfig.duckWheelRed) {
            robot.towerMotor.setPower(-Constants.towerWheelSpeedEndgame);
        } else {
            robot.towerMotor.setPower(0);
        }

        // Lift motor
        if (ControlConfig.linSlideSlow) {
            robot.liftMotor.setPower(1.0);
        } else if (ControlConfig.linSlideSlow) {
            // FIXME this doesn't set dumpservo to the collect position
            robot.dumpServo.setPosition(Constants.collectPosition);
            robot.liftMotor.setPower(-1.0);
        } else {
            robot.liftMotor.setPower(0);
        }

        // Collection motor
        if (ControlConfig.collectWheel) {
            robot.collectionMotor.setPower(1.0);
        } else if (ControlConfig.unCollectWheel) {
            robot.collectionMotor.setPower(-1.0);
        } else {
            robot.collectionMotor.setPower(0);
        }
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

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });

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