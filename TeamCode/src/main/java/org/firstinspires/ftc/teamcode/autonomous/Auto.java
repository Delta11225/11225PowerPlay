package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware23;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(preselectTeleOp = "TeleopFinal")
public class Auto extends LinearOpMode {
    FtcDashboard ftcDashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        // This lets us write telemetry to both FTC Dashboard and driverhub
//        telemetry = new MultipleTelemetry(telemetry, ftcDashboard.getTelemetry());
//        telemetry.setAutoClear(true);
        Hardware23 robot = new Hardware23(hardwareMap);

        // Init the camera. Save pipeline, as we need it later
        DetectionPipeline pipeline = new DetectionPipeline();
        OpenCvCamera webcam = initCamera(pipeline);

        initMotors(robot);
//        int liftEncoderStart = robot.liftMotor.getCurrentPosition();

        // We only need the delay here, everything else just gets passed to TrajectoryGenerator
        AutoState autoState = getUserInput();
        Constants.matchState = autoState;
        long delay = autoState.delay;

        telemetry.addLine("Building trajectories...");
        telemetry.update();
        TrajectoryGenerator trajGen = new TrajectoryGenerator(robot, autoState, telemetry);
        telemetry.addData("Built! Time taken (s)", getRuntime());
        telemetry.update();

        // Ask the driver if they missed anything
        confirmAutoGood();

        resetRuntime();

        // Loop for user friendliness, constantly updates camera detection result
        while (!isStopRequested() && !isStarted()) {
            // Confirm auto settings
            telemetry.addLine("Init complete, ready to run");
            telemetry.addData("Color", autoState.color);
            telemetry.addData("Start pos", autoState.position);
            telemetry.addData("Delay", delay);
            telemetry.addData("Camera detection", pipeline.getLastPos());
            telemetry.update();
        }

        // Just to be safe
        if (isStopRequested()) {
            return;
        }

        waitForStart();
        // If we need to keep track of time
        resetRuntime();

        Constants.linearSlideZeroOffset = robot.linearSlide.getCurrentPosition();

        // Add parking pos to telemetry to we don't have to worry and save last pos
        ParkingPosition parkPos = pipeline.getLastPos();
        telemetry.clearAll();
        telemetry.addData("Park pos", parkPos);
        telemetry.addData("Time taken (s)", getRuntime());
        telemetry.update();

        // To prevent issues, stop the camera stream
//        resetRuntime();
        webcam.stopStreaming();
        ftcDashboard.stopCameraStream();
        telemetry.addData("Time taken (s)", getRuntime());
        telemetry.update();

//        resetRuntime();
        TrajectorySequence[] trajSequences;
        trajSequences = trajGen.getAppropriateTrajectory(autoState, parkPos);
        telemetry.addData("Time taken (s)", getRuntime());
        telemetry.update();

        // Start of game delay, if we need it
        sleep(delay);

        for (TrajectorySequence trajSeq : trajSequences) {
            robot.drive.followTrajectorySequence(trajSeq);
        }

        Constants.currentPose = robot.drive.getPoseEstimate();
    }

    private OpenCvCamera initCamera(OpenCvPipeline pipeline) {
        // Open the camera, stream to FTC and driver hub. Attach given pipeline
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                webcam.setPipeline(pipeline);
                ftcDashboard.startCameraStream(webcam, 0);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        return webcam;
    }

    public AutoState getUserInput() {
        // Get all the user input to save into auto state
        // To solve bugs
        gamepad2.reset();

        // Red or blue side?
        Color color = getColor();
        sleep(100);

        // Front or back?
        StartPosition startPosition = getPosition();
        sleep(100);

        long delay = getDelay();
        sleep(100);

        return new AutoState(color, startPosition, delay);
    }

    private Color getColor() {
        // Need two things for normal or dualshock controllers
        telemetry.clear();
        telemetry.addData("Color?","");
        telemetry.addData("Blue", "X or Square");
        telemetry.addData("Red", "B or Circle");
        telemetry.update();

        Color color = null;

        // Delay until we get a result or a stop is requested. Otherwise robot gets stuck
        while (!isStopRequested()) {
            // This is for some reason required as otherwise we can only run autonomous once
            // before we need to restart the robot. Included at the beginning of each while loop
            gamepad2.toString();
            if (gamepad2.x) {
                color = Color.BLUE;
                break;
            } else if (gamepad2.b) {
                color = Color.RED;
                break;
            }
        }
        telemetry.addLine("Color confirmed, " + color);
        telemetry.update();

        return color;
    }

    private StartPosition getPosition() {
        telemetry.addLine("Position");
        telemetry.addLine("Front = dpad Up");
        telemetry.addLine("Back = dpad Down");
        telemetry.update();

        StartPosition startPosition = null;

        while (!isStopRequested()) {
            gamepad2.toString();
            if (gamepad2.dpad_up) {
                startPosition = StartPosition.FRONT;
                break;
            } else if (gamepad2.dpad_down) {
                startPosition = StartPosition.BACK;
                break;
            }
        }

        telemetry.addLine("Position confirmed, " + startPosition);
        telemetry.update();

        return startPosition;
    }

    private long getDelay() {
        // To make the user press and unpress the button, as otherwise increases really fast
        boolean buttonUnpressed = true;
        long delay = 0;

        while (!isStopRequested()) {
            gamepad2.toString();
            telemetry.addData("Delay? RB to add 10 ms, LB to add 100ms, y or Triangle done", delay);
            telemetry.update();
            if (gamepad2.right_bumper && buttonUnpressed) {
                delay += 10;
                buttonUnpressed = false;
            } else if (gamepad2.left_bumper && buttonUnpressed) {
                delay += 100;
                buttonUnpressed = false;
            } else if (gamepad2.y) {
                break;
            } else if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
                buttonUnpressed = true;
            }
        }
        telemetry.addLine("Delay confirmed, " + delay);
        telemetry.update();
        return delay;
    }

    private void confirmAutoGood() {
        telemetry.addLine("Did you check camera stream? Press b or Circle");
        telemetry.update();
        // Delay until they press the button
        while (!isStopRequested() && !gamepad2.b) {}

        telemetry.addLine("Did you preload a cone? Press a or X");
        telemetry.update();
        while (!isStopRequested() && !gamepad2.a) {}

        telemetry.addLine("Is the linear slide down? Press x or Square");
        telemetry.update();
        while (!isStopRequested() && !gamepad2.x) {}
    }

    private void initMotors(Hardware23 robot) {
        return;
    }
}