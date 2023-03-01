package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware23;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

// For here, after auto ends, automatically selects our teleop
@Autonomous(preselectTeleOp = "TeleopFinal")
public class Auto extends LinearOpMode {
    // Fetch an instance of FTCDashboard for camera display later
    FtcDashboard ftcDashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize all our robots hardware. We do this first as it is most important
        Hardware23 robot = new Hardware23(hardwareMap);

        // Init the camera. Save the pipeline, as we need it for results later
        DetectionPipeline pipeline = new DetectionPipeline();
        OpenCvCamera webcam = initCamera(pipeline);

        // This method asks the user for the auto state and packages it into a nice class
        AutoState autoState = getUserInput();
        // Save current auto state in constats for teleop
        Constants.matchState = autoState;
        // We only need to get the delay here, everything else is handled by TrajectoryGenerator
        long delay = autoState.delay;

        // Get the robot's start offset, as provided by the user
        Vector2d startOffset = getUserOffset(autoState);

        telemetry.addLine("Building trajectories...");
        telemetry.update();
        // Generate the appropriate trajectory
        TrajectoryGenerator trajGen = new TrajectoryGenerator(robot, autoState, startOffset, telemetry);
        telemetry.addData("Built! Time taken (s)", getRuntime());
        telemetry.update();

        // Ask the driver if they missed anything
        confirmAutoGood();

        // Loop to tell user current auto, constantly updates camera detection result
        while (!isStopRequested() && !isStarted()) {
            // Confirm auto settings
            telemetry.addLine("Init complete, ready to run");
            telemetry.addData("Color", autoState.color);
            telemetry.addData("Start pos", autoState.position);
            telemetry.addData("Auto Type", autoState.autoType);
            telemetry.addData("Delay", delay);
            telemetry.addData("Offset", startOffset);
            telemetry.addData("Camera detection", pipeline.getLastPos());
            telemetry.update();
        }

        // Just to be safe
        if (isStopRequested()) {
            return;
        }

        // Wait until start pressed
        waitForStart();

        // If we need to keep track of time
        resetRuntime();

        // Set linear slide zero to current linear slide position in case it has been moved
        // during autonomous init
        Constants.linearSlideZeroOffset = robot.linearSlide.getCurrentPosition();

        // Save first detected parking position before we run and add to telemetry so we don't
        // update parking position while we are running
        ParkingPosition parkPos = pipeline.getLastPos();
        telemetry.clearAll();
        telemetry.addData("Park pos", parkPos);
        telemetry.update();

        // To maybe prevent some memory issues, stop the camera stream as we don't need it anyway
        webcam.stopStreaming();
        ftcDashboard.stopCameraStream();

        // Get the trajectories we need to run for this autonomous
        TrajectorySequence[] trajSequences;
        trajSequences = trajGen.getAppropriateTrajectory(parkPos);

        // Execute start of game delay
        sleep(delay);

        // Follow each trajectory sequence we have specified
        for (TrajectorySequence trajSeq : trajSequences) {
            robot.drive.followTrajectorySequence(trajSeq);
        }

        // Save the last pose for teleop
        Constants.currentPose = robot.drive.getPoseEstimate();
    }

    /**
     * Prompt the user for the start offset of the robot
     * @param autoState The auto state, used for auto color
     * @return The user generated start offset
     */
    private Vector2d getUserOffset(AutoState autoState) {
        // Used to prevent triggering multiple times when button is pushed but not released
        boolean buttonUnpressed = true;
        // Holds current offset
        Vector2d offset = new Vector2d(0, 0);

        // Modifier is how much the pose will adjust per button press, and is reversed for red positions
        // but only on left-right
        double modifier = 0.1;
        if (autoState.color == Color.RED) {
            modifier *= -1;
        }

        while (!isStopRequested()) {
            gamepad2.toString();
            telemetry.addData("Start offset? use dpad. Circle done", offset.toString());
            telemetry.update();

            // Basically just adjust offset based which button is pushed. Take absolute value
            // of modifier for y adjustments as they do not change for red or blue
            if (gamepad2.dpad_up && buttonUnpressed) {
                offset = new Vector2d(offset.getX(), offset.getY() + Math.abs(modifier));
                buttonUnpressed = false;
            } else if (gamepad2.dpad_down && buttonUnpressed) {
                offset = new Vector2d(offset.getX(), offset.getY() - Math.abs(modifier));
                buttonUnpressed = false;
            } else if (gamepad2.dpad_left) {
                offset = new Vector2d(offset.getX() - modifier, offset.getY());
                buttonUnpressed = false;
            } else if (gamepad2.dpad_right) {
                offset = new Vector2d(offset.getX() + modifier, offset.getY());
                buttonUnpressed = false;
            } else if (gamepad2.circle) {
                break;
            } else if (!(gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right)) {
                buttonUnpressed = true;
            }
        }
        telemetry.addLine("Offset confirmed");
        telemetry.update();
        return offset;
    }

    /**
     * Initalize the webcam and attach an image processing pipeline
     * @param pipeline The image processing pipeline to attach
     * @return The initalized camera
     */
    private OpenCvCamera initCamera(OpenCvPipeline pipeline) {
        // This line gets the internal id of the camera so we can get it with another method
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // OpenCvCameraFactory will initalize the webcam for us, and we just need to pass it the webcam
        // object, which we can get from the ID we just found
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(
                WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // We have to open it asynchronously as opposed to synchronously because for some reason
        // the synchronous method is deprecated
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // When the webcam opens normally, start streaming to the driver hub and FTC dashboard
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                webcam.setPipeline(pipeline);
                // The 0 max FPS means no max FPS
                ftcDashboard.startCameraStream(webcam, 0);
            }

            @Override
            public void onError(int errorCode) {
                // If it throws an error while opening the camera, we log it, but otherwise ignore it
                Log.e("[Camera]", "Error while opening camera, error code " + errorCode);
                telemetry.addLine("Error while opening camera, error code " + errorCode);
                telemetry.update();
            }
        });

        return webcam;
    }

    /**
     * Get autonomous state from user
     * @return The specified autonomous state
     */
    public AutoState getUserInput() {
        // Get all the user input to save into auto state
        // To solve bugs
        gamepad2.reset();

        // Red or blue side?
        Color color = getColor();
        // Sleep to allow time for telemetry to show up
        sleep(100);

        // Front or back?
        StartPosition startPosition = getPosition();
        sleep(100);

        // Long or Sweat?
        AutoType autoType = getAutoType();

        // Delay for how long?
        long delay = getDelay();
        sleep(100);

        return new AutoState(color, startPosition, autoType, delay);
    }

    /**
     * Ask user for auto color
     * @return User specified autonomous color
     */
    private Color getColor() {
        // Need two buttons in message for normal or dualshock controllers
        telemetry.clear();
        telemetry.addData("Color?","");
        telemetry.addData("Blue", "X or Square");
        telemetry.addData("Red", "B or Circle");
        telemetry.update();

        // Save color
        Color color = null;

        // Delay until we get a result or a stop is requested. Otherwise robot gets stuck
        while (!isStopRequested()) {
            // This is for some reason required as otherwise we can only run autonomous once
            // before we need to restart the robot sometimes. Included at the beginning of each while loop
            gamepad2.toString();

            // Wait for user to specify color and save it, then break out of loop
            if (gamepad2.x) {
                color = Color.BLUE;
                break;
            } else if (gamepad2.b) {
                color = Color.RED;
                break;
            }
        }
        // Confirm to user specified color
        telemetry.addLine("Color confirmed, " + color);
        telemetry.update();

        return color;
    }

    /**
     * Ask user for start position
     * @return User specified auto start position
     */
    private StartPosition getPosition() {
        // Same logic as getColor
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

    /**
     * Ask user for auto type
     * @return User specified type of autonomous
     */
    private AutoType getAutoType() {
        // Same logic as getColor
        telemetry.clear();
        telemetry.addData("Auto Type?","");
        telemetry.addData("Blue", "X or Square");
        telemetry.addData("Red", "B or Circle");
        telemetry.update();

        AutoType autoType = null;

        // Delay until we get a result or a stop is requested. Otherwise robot gets stuck
        while (!isStopRequested()) {
            // This is for some reason required as otherwise we can only run autonomous once
            // before we need to restart the robot. Included at the beginning of each while loop
            gamepad2.toString();
            if (gamepad2.x) {
                autoType = AutoType.LONG;
                break;
            } else if (gamepad2.b) {
                autoType = AutoType.SWEAT;
                break;
            }
        }
        telemetry.addLine("Auto Type confirmed, " + autoType);
        telemetry.update();

        return autoType;
    }

    /**
     * Ask user for auto delay
     * @return User specified amount for how long to delay at start of auto
     */
    private long getDelay() {
        // To make the user press and unpress the button to add delay, as otherwise delay increases really fast
        boolean buttonUnpressed = true;
        long delay = 0;

        // Increment delay by correct increment every loop, and break when done
        while (!isStopRequested()) {
            gamepad2.toString();
            telemetry.addData("Delay? RB to add 10 ms, LB to add 100ms, y or Triangle done", delay);
            telemetry.update();
            if (gamepad2.right_bumper && buttonUnpressed) {
                delay += 10;
                // Set so user must release button, as otherwise delay increases really fast
                buttonUnpressed = false;
            } else if (gamepad2.left_bumper && buttonUnpressed) {
                delay += 100;
                buttonUnpressed = false;
            } else if (gamepad2.y) {
                break;
            } else if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
                // If buttons are released
                buttonUnpressed = true;
            }
        }
        telemetry.addLine("Delay confirmed, " + delay);
        telemetry.update();
        return delay;
    }

    /**
     * Prompts the user with on screen telemetry to ask if they have completed pre autonomous checks
     */
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
}