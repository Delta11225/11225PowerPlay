package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class TeleopTestSound extends LinearOpMode {
    boolean isX = false;
    boolean wasX = false;
    boolean bruhFound = false;
    int bruhID = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        bruhID = hardwareMap.appContext.getResources().getIdentifier("bruh_mp3", "raw", hardwareMap.appContext.getPackageName());

        if (bruhID != 0) {
            bruhFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, bruhID);
        }

        telemetry.addData("sound resource", bruhFound ? "Found" : "NOT found\n Add bruh.mp3 to /src/main/res/raw" );

        telemetry.addData(">", "Press X to play sounds.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            isX = gamepad1.x;
            if (bruhFound && isX && !wasX) {
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, bruhID);
                telemetry.addData("Playing", "bruh");
                telemetry.update();
            }

            wasX = isX;
        }
    }
}
