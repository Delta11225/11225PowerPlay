package org.firstinspires.ftc.teamcode.teleop.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class TeleopTestGamepadCallback extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        GamepadEx test1 = new GamepadEx(gamepad1);
        ToggleButtonReader toggleA = new ToggleButtonReader(test1, GamepadKeys.Button.A);

        waitForStart();

        while (opModeIsActive()) {
            if (toggleA.getState()) {
                telemetry.addData("State", "yes");
            } else {
                telemetry.addData("State", "no");
            }
            telemetry.update();
            toggleA.readValue();
        }
    }
}
