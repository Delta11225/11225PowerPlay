package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Hardware22;

@Autonomous
@Disabled
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware22 robot = new Hardware22(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            robot.frontLeft.setPower(1);
        }
    }
}
