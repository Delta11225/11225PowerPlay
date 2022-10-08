package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class ServoTest extends OpMode {
    Servo leftClaw;
    Servo rightClaw;

    @Override
    public void init() {
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            leftClaw.setPosition(1);
        } else if (gamepad1.y){
            leftClaw.setPosition(0);
        }

        if (gamepad1.x) {
            rightClaw.setPosition(1);
        } else if (gamepad1.b) {
            rightClaw.setPosition(0);
        }
    }
}
