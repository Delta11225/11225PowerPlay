package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware22;
import org.firstinspires.ftc.teamcode.teleop.competition.ControlConfig;

@TeleOp
@Disabled
public class DuckWheelTest extends LinearOpMode {
    double duckWheelMaxSpeed = Constants.towerWheelSpeed;
    Hardware22 robot;
    boolean buttonUnpressed = true;
    double duckWheelSpeed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware22(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        telemetry.update();

        robot.towerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            ControlConfig.update(gamepad1, gamepad2);

            if (ControlConfig.duckWheelBlue) {
                robot.towerMotor.setPower(duckWheelSpeed);

                duckWheelSpeed += .5;
                if (duckWheelSpeed > duckWheelMaxSpeed) {
                    duckWheelSpeed = duckWheelMaxSpeed;
                }
            } else {
                robot.towerMotor.setPower(0);
                duckWheelSpeed = 0;
            }

            telemetry.addData("Duck speed", duckWheelMaxSpeed);
            telemetry.update();

            if (gamepad2.y && buttonUnpressed) {
                duckWheelMaxSpeed += 0.02;
                buttonUnpressed = false;
            } else if (gamepad2.a && buttonUnpressed) {
                duckWheelMaxSpeed -= 0.02;
                buttonUnpressed = false;
            } else if (!gamepad2.y && !gamepad2.a) {
                buttonUnpressed = true;
            }
        }
    }
}
