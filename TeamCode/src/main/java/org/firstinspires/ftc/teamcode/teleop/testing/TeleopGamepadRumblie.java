package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class TeleopGamepadRumblie extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Touchpad finger 1 x", gamepad1.touchpad_finger_1_x);
            telemetry.addData("Touchpad finger 1 y", gamepad1.touchpad_finger_1_y);
            telemetry.addData("Touchpad finger 2 x", gamepad1.touchpad_finger_2_x);
            telemetry.addData("Touchpad finger 2 y", gamepad1.touchpad_finger_2_y);
            telemetry.addData("Gamepad", gamepad1.toString());
            telemetry.update();
        }
    }
}
