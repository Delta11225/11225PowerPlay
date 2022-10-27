package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp
public class ColorSensorTest extends OpMode {
    NormalizedColorSensor sensor;

    @Override
    public void init() {
        sensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
    }

    @Override
    public void loop() {
        telemetry.addData("Color vals, r", sensor.getNormalizedColors().red);
        telemetry.addData("Color vals, g", sensor.getNormalizedColors().green);
        telemetry.addData("Color vals, b", sensor.getNormalizedColors().blue);
        telemetry.addData("Color vals, a", sensor.getNormalizedColors().alpha);
        telemetry.update();
    }
}
