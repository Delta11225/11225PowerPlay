package org.firstinspires.ftc.teamcode.testing;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.ControlConfig;
import org.firstinspires.ftc.teamcode.util.Hardware23;

@TeleOp
//@Disabled
public class LinearSlideTest extends LinearOpMode {

    DcMotor linearSlide;
    Servo rightClaw;
    Servo leftClaw;
    int holdPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Hardware23 robot = new Hardware23(hardwareMap);
        linearSlide = robot.linearSlide;
//        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");

        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        leftClaw = hardwareMap.get(Servo.class, "left_claw");

        telemetry.addData("status", "Initialized");
        telemetry.update();

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // puts the motor in brake setting so that when motor power = 0 the motor will hold position instead of idling
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //puts the motor in run using encoder mode so that motors can run while tracking encoder values
        //Must have this line after stop/reset encoders or motor can't run
        linearSlide.setTargetPosition(0);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            linearSlide.setTargetPosition(holdPosition);
            /////////////////////////////LINEAR SLIDE//////////////////////////////
            if (ControlConfig.liftSlide) {
                holdPosition += 10;
            } else if (ControlConfig.lowerSlide) {
                holdPosition -= 10;
            } else {
                holdPosition = linearSlide.getCurrentPosition();
            }
            telemetry.addData("encoder", linearSlide.getCurrentPosition());
            Log.d("LinEncoder", String.valueOf(linearSlide.getCurrentPosition()));
            telemetry.update();

////////////////////GRABBER////////////////////////////////////////////////////////

            // A button = open claw, b button = closed claw
            if (gamepad1.a) {
                rightClaw.setPosition(Constants.rightClawOpen); // Right claw open
                leftClaw.setPosition(Constants.leftClawOpen); // Left claw open
            }
            if (gamepad1.b) {
                rightClaw.setPosition(Constants.rightClawClosed); // Right claw closed
                leftClaw.setPosition(Constants.leftClawClosed); // Left claw closed
            }


        }
    }
}

