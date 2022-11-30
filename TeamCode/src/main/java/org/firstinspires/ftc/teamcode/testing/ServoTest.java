package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//@Disabled
@TeleOp
@Config
public class ServoTest extends OpMode {
    Servo leftClaw;
    Servo rightClaw;
    public static double step = 0.001;

    @Override
    public void init() {
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            leftClaw.setPosition(leftClaw.getPosition() + step);
        } else if (gamepad1.y){
            leftClaw.setPosition(leftClaw.getPosition() - step);
        }

        if (gamepad1.x) {
            rightClaw.setPosition(rightClaw.getPosition() + step);
        } else if (gamepad1.b) {
            rightClaw.setPosition(rightClaw.getPosition() - step);
        }
    }
}
