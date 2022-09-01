package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class TelemetryTest extends OpMode {
    @Override
    public void init() {
        telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
    }

    @Override
    public void loop() {
        telemetry.addLine("<b><p " +
                "style='color: #daa052;'>" +
                "TEST" +
                "</p></b>");
        telemetry.addLine("test2");
    }
}
