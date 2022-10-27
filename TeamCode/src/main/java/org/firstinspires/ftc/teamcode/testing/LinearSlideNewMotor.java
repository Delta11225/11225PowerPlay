package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class LinearSlideNewMotor extends LinearOpMode {

    DcMotor linearSlide;
    Servo rightClaw;
    Servo leftClaw;
    int holdPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");

        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        leftClaw = hardwareMap.get(Servo.class, "left_claw");

        telemetry.addData("status", "Initialized");
        telemetry.update();

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // puts the motor in brake setting so that when motor power = 0 the motor will hold position instead of idling
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //puts the motor in run using encoder mode so that motors can run while tracking encoder values
        //Must have this line after stop/reset encoders or motor can't run
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            /////////////////////////////LINEAR SLIDE//////////////////////////////
            if (gamepad1.dpad_up && linearSlide.getCurrentPosition() < 1555) {
                linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                linearSlide.setPower(0.5);
            } else if (gamepad1.dpad_down && linearSlide.getCurrentPosition() > 0) {
                linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                linearSlide.setPower(-0.4);
            } else {
                linearSlide.setPower(0);
                linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                telemetry.addData("encoder", linearSlide.getCurrentPosition());
                telemetry.update();


////////////////////GRABBER////////////////////////////////////////////////////////

            // A button = open claw, b button = closed claw
            if (gamepad1.a) {
                rightClaw.setPosition(0.95); // Right claw open
                leftClaw.setPosition(0.0); // Left claw open
            }
            if (gamepad1.b) {
                rightClaw.setPosition(0.70); // Right claw closed
                leftClaw.setPosition(0.25); // Left claw closed
            }


        }
    }
}