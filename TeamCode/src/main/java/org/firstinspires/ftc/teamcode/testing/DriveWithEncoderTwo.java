package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
@Disabled
@Deprecated
public class DriveWithEncoderTwo extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;

    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double WHEEL_DIAMETER_CM = 10;
    static final double ENCODER_COUNTS_PER_CM = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_CM);

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        encoderDrive(0.05, 60);
    }

    public void encoderDrive(double speed, double cm) {

        int newLeftTarget;
        if (opModeIsActive()) {
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (cm * ENCODER_COUNTS_PER_CM);

            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newLeftTarget);

            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftMotor.setPower(speed);
            rightMotor.setPower(speed);

            while (opModeIsActive() && leftMotor.isBusy()) {}

            leftMotor.setPower(0);
            rightMotor.setPower(0);

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Uncomment line below to pause a bit between movements
            // sleep(10);
        }
    }
}
