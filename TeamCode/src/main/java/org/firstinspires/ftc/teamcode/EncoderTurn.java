package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class EncoderTurn extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;

    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double WHEEL_DIAMETER_CM = 10;
    static final double ENCODER_COUNTS_PER_CM = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_CM);
    static final double WHEEL_SEPERATION = 42.7;

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

        encoderTurn(0.25, -90, true );
        sleep(10000);
    }

    public void encoderTurn(double speed, double degrees, boolean isPointTurn){
        if (degrees < 0) {
            speed = -speed;
        }
        double turningCircumference = Math.PI * WHEEL_SEPERATION;
        if(!isPointTurn){
            turningCircumference *= 2;
        }
        telemetry.addData("Turning Circumference", turningCircumference);
        double encoderTarget = ENCODER_COUNTS_PER_CM * turningCircumference * (degrees/360);
        telemetry.addData("Encoder Target", encoderTarget);


        leftMotor.setTargetPosition((int) encoderTarget);
        rightMotor.setTargetPosition((int) -encoderTarget);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(speed);
        if (isPointTurn) {
            rightMotor.setPower(-speed);
        }

        while (opModeIsActive() && leftMotor.isBusy()) {
            telemetry.update();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}
