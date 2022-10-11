package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Hardware22;
import org.firstinspires.ftc.teamcode.util.Hardware23;

@TeleOp
@Disabled
public class LinearSlideEncoderMaxTest extends OpMode {
    Hardware23 robot = null;

    @Override
    public void init() {
        robot = new Hardware23(hardwareMap);

        robot.linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            robot.linearSlide.setPower(0.1);
        } else if (gamepad1.dpad_down) {
            robot.linearSlide.setPower(-0.1);
        } else {
            robot.linearSlide.setPower(0);
        }
        telemetry.addData("Lift encoders", robot.linearSlide.getCurrentPosition());
    }
}
