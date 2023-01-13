package org.firstinspires.ftc.teamcode.testing;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxModuleIntf;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware23;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

@TeleOp
public class TrackMotorPowerDraw extends LinearOpMode {
    private int holdPosition;
    LynxModuleIntf expansionHub;

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
            /////////////////////////////LINEAR SLIDE//////////////////////////////
            if (gamepad1.dpad_up && linearSlide.getCurrentPosition() < Constants.getLiftEncoderMax()) {
                linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                linearSlide.setPower(0.5);
            } else if (gamepad1.dpad_down && linearSlide.getCurrentPosition() > 0) {
                linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                linearSlide.setPower(-0.5);
            } else {
                linearSlide.setPower(0);
            }
            telemetry.addData("Motor current draw", getExpansionHubMotorCurrentDraw(3));
            Log.d("Power", String.valueOf(getExpansionHubMotorCurrentDraw(3)));
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
