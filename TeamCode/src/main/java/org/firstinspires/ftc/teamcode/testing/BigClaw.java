package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class BigClaw extends LinearOpMode {

    DcMotor linearSlide;
    Servo rightClaw;
    Servo leftClaw;
    int holdPosition;
    @Override
    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        leftClaw = hardwareMap.get(Servo.class, "left_claw");

        telemetry.addData("status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "running");
            telemetry.addData("left claw", leftClaw.getPosition());
            telemetry.addData("right claw", rightClaw.getPosition());
            telemetry.update();
            // A button = open claw, b button = closed claw
            if (gamepad1.a) {
//                rightClaw.setPosition(0.44); // Right claw open
                leftClaw.setPosition(0.48); // Left claw open
            }
            if (gamepad1.b) {
//                rightClaw.setPosition(0.6); // Right claw closed
                rightClaw.setPosition(0.64); // Left claw closed
            }


        }
    }
}