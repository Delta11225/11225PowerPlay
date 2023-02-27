package org.firstinspires.ftc.teamcode.testing;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.code.SymbolMetadata;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware23;
import org.firstinspires.ftc.teamcode.util.SizedStack;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;


// 2/21 Honestly this will be obsoleted by belt fed linear slide so idk why im doing this

// NVM, i am working on something that might work maybe

// IMPORTANT this has been declared as currently impossible with a string-fed linear slide.
// The intention of this class was to track the motor's power draw to see if we could detect
// when it was stuck, but since current spikes when the linear slide suddenly changes direction
// it is very difficult to detect whether the current spike is due to the slide being stuck
// or to the slide changing direction. There is probably a way to do it, but I don't care and it
// probably doesn't matter right now. I will wait until we get a belt-fed slide.
@Deprecated
@TeleOp
@Disabled
public class TrackMotorPowerDraw extends LinearOpMode {
    private int holdPosition;
    LynxModuleIntf expansionHub;
    double maxPowerDraw = 0;
    private ElapsedTime slideCooldown = new ElapsedTime();
    private SizedStack<Integer> lastControls = new SizedStack<>(50);
    private SizedStack<Double> lastPowerVals = new SizedStack<>(50);

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorController realController = hardwareMap.dcMotorController.iterator().next();

        // To access the current sensors, we need an instance of the Expansion Hub (LynxModule)
        // itself, or a PretendLynxModule. We can get this through the LynxDcMotorController,
        // but only through reflection.
        Method getModule_method;

        // The "getModule" method is located within LynxDcMotorController's parent
        // class, LynxController
        try {
            getModule_method = LynxController.class.getDeclaredMethod("getModule");

            getModule_method.setAccessible(true);

            // Actually get the value from the controller that was passed in. We cast this to
            // a LynxModuleIntf instead of a LynxModule to support the use of a PretendLynxModule.
            expansionHub = (LynxModuleIntf) getModule_method.invoke(realController);
        } catch (NoSuchMethodException | InvocationTargetException | IllegalAccessException e) {
//            e.printStackTrace();
        }

        Hardware23 robot = new Hardware23(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            DcMotor linearSlide = robot.linearSlide;
            if (slideCooldown.seconds() < 2) {
                lastControls.push(0);
                lastPowerVals.push(0d);
                linearSlide.setPower(0);
                continue;
            }
            /////////////////////////////LINEAR SLIDE//////////////////////////////
            if (gamepad1.dpad_up && linearSlide.getCurrentPosition() < Constants.getLiftEncoderMax()) {
                lastControls.push(1);
                linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                linearSlide.setPower(0.5);
            } else if (gamepad1.dpad_down && linearSlide.getCurrentPosition() > 0) {
                lastControls.push(-1);
                linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                linearSlide.setPower(-0.5);
            } else {
                lastControls.push(0);
                linearSlide.setPower(0);
            }
            double powerDraw = getExpansionHubMotorCurrentDraw(3);
            lastPowerVals.push(powerDraw);
            telemetry.addData("Motor current draw", powerDraw);
            Log.d("Power", String.valueOf(powerDraw));

            if (powerDraw > maxPowerDraw) {
                maxPowerDraw = powerDraw;
            }

            double avg = lastPowerVals.stream()
                    .mapToDouble(Double::valueOf)
                    .average()
                    .orElse(0);

            double controlAvg = lastControls.stream()
                    .mapToInt(Integer::valueOf)
                    .average()
                    .orElse(0);
            telemetry.addData("Power avg", avg);
            telemetry.addData("Control avg", controlAvg);
            if (avg >= 1700 && (controlAvg == -1 || controlAvg == 1)) {
                slideCooldown.reset();
            }
            Log.d("PowerMax", String.valueOf(maxPowerDraw));
            telemetry.update();

        }
    }

    // FIXME this is not working RN
    public synchronized double getExpansionHubMotorCurrentDraw(int port) {
        LynxGetADCCommand.Channel channel;

        if (port == 0) {
            channel = LynxGetADCCommand.Channel.MOTOR0_CURRENT;
        } else if (port == 1) {
            channel = LynxGetADCCommand.Channel.MOTOR1_CURRENT;
        } else if (port == 2) {
            channel = LynxGetADCCommand.Channel.MOTOR2_CURRENT;
        } else if (port == 3) {
            channel = LynxGetADCCommand.Channel.MOTOR3_CURRENT;
        } else {
            return -1; // TODO: Should we handle an invalid port with a crash?
        }

        LynxGetADCCommand command = new LynxGetADCCommand(expansionHub, channel, LynxGetADCCommand.Mode.ENGINEERING);
        try {
            LynxGetADCResponse response = command.sendReceive();
            return response.getValue();
        } catch (InterruptedException | RuntimeException | LynxNackException e) {
//            handleException(e);
        }
        return -1;
    }
}
