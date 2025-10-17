package org.firstinspires.ftc.teamcode.external.samples

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.teamcode.internal.RobotOpModeBase
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.concurrent.TimeUnit

// Configuration constants
const val DESIRED_DISTANCE = 12.0  // How close the camera should get to the target (inches)
const val SPEED_GAIN = 0.02        // Forward speed control gain
const val STRAFE_GAIN = 0.015      // Strafe speed control gain
const val TURN_GAIN = 0.01         // Turn control gain
const val MAX_AUTO_SPEED = 0.5     // Clip the approach speed to this max value
const val MAX_AUTO_STRAFE = 0.5    // Clip the strafing speed to this max value
const val MAX_AUTO_TURN = 0.3      // Clip the turn speed to this max value
const val USE_WEBCAM = true        // Set true to use a webcam, or false for a phone camera
const val DESIRED_TAG_ID = -1      // Choose the tag you want to approach or set to -1 for ANY tag

/**
 * Omni Drive To AprilTag - Kotlin Iterative Implementation
 *
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be
 * directly in front of the tag, and driving towards the tag to achieve the desired distance.
 *
 * Controls:
 * - Left stick: Forward/back & left/right (manual mode)
 * - Right stick: Rotation (manual mode)
 * - Left bumper: Hold to enable automatic "Drive to target" mode
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame
 * 2) Strafe the robot towards the centerline of the Tag
 * 3) Drive towards the Tag to get to the desired distance
 */
@TeleOp(name = "Omni Drive To AprilTag", group = "Concept")
@Disabled
class AprilTagOmniDrive : RobotOpModeBase() {
    // Motors
    val frontLeftDrive: DcMotor by hardware("front_left_drive")
    val frontRightDrive: DcMotor by hardware("front_right_drive")
    val backLeftDrive: DcMotor by hardware("back_left_drive")
    val backRightDrive: DcMotor by hardware("back_right_drive")

    // Vision
    private lateinit var visionPortal: VisionPortal
    private lateinit var aprilTag: AprilTagProcessor

    // State for camera initialization
    private enum class CameraState { NOT_STARTED, WAITING, READY }
    private var cameraState = CameraState.NOT_STARTED

    override fun init() {
        // Configure motor directions
        frontLeftDrive.direction = DcMotorSimple.Direction.REVERSE
        backLeftDrive.direction = DcMotorSimple.Direction.REVERSE
        frontRightDrive.direction = DcMotorSimple.Direction.FORWARD
        backRightDrive.direction = DcMotorSimple.Direction.FORWARD

        // Initialize AprilTag detection
        initAprilTag()

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream")
        telemetry.addData(">", "Robot Ready. Press START.")
        telemetry.update()
    }

    override fun start() {
        super.start()
        // Start camera exposure configuration
        if (USE_WEBCAM) {
            cameraState = CameraState.WAITING
        }
    }

    override fun loop() {
        // Handle camera exposure setup (non-blocking)
        updateCameraExposure()

        // Look for AprilTags
        var targetFound = false
        var desiredTag: AprilTagDetection? = null

        for (detection in aprilTag.detections) {
            // Look to see if we have size info on this tag
            if (detection.metadata != null) {
                // Check to see if we want to track towards this tag
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    targetFound = true
                    desiredTag = detection
                    break
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id)
                }
            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id)
            }
        }

        // Display what we see
        if (targetFound) {
            telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n")
            desiredTag?.let { tag ->
                telemetry.addData("Found", "ID %d (%s)", tag.id, tag.metadata.name)
                telemetry.addData("Range", "%5.1f inches", tag.ftcPose.range)
                telemetry.addData("Bearing", "%3.0f degrees", tag.ftcPose.bearing)
                telemetry.addData("Yaw", "%3.0f degrees", tag.ftcPose.yaw)
            }
        } else {
            telemetry.addData("\n>", "Drive using joysticks to find valid target\n")
        }

        // Calculate drive, strafe, and turn values
        val drive: Double
        val strafe: Double
        val turn: Double

        if (gamepad1.left_bumper && targetFound && desiredTag != null) {
            // Automatic mode - drive to target
            val rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE
            val headingError = desiredTag.ftcPose.bearing
            val yawError = desiredTag.ftcPose.yaw

            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED)
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN)
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE)

            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f", drive, strafe, turn)
        } else {
            // Manual mode - use joysticks
            drive = -gamepad1.left_stick_y.toDouble() / 2.0   // Reduce drive rate to 50%
            strafe = -gamepad1.left_stick_x.toDouble() / 2.0  // Reduce strafe rate to 50%
            turn = -gamepad1.right_stick_x.toDouble() / 3.0   // Reduce turn rate to 33%
            telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f", drive, strafe, turn)
        }

        telemetry.update()

        // Apply desired axes motions to the drivetrain
        moveRobot(drive, strafe, turn)
    }

    override fun stop() {
        visionPortal.close()
    }

    /**
     * Move robot according to desired axes motions
     * Positive X is forward
     * Positive Y is strafe left
     * Positive Yaw is counter-clockwise
     */
    private fun moveRobot(x: Double, y: Double, yaw: Double) {
        // Calculate wheel powers
        var frontLeftPower = x - y - yaw
        var frontRightPower = x + y + yaw
        var backLeftPower = x + y - yaw
        var backRightPower = x - y + yaw

        // Normalize wheel powers to be less than 1.0
        val max = maxOf(
            Math.abs(frontLeftPower),
            Math.abs(frontRightPower),
            Math.abs(backLeftPower),
            Math.abs(backRightPower)
        )

        if (max > 1.0) {
            frontLeftPower /= max
            frontRightPower /= max
            backLeftPower /= max
            backRightPower /= max
        }

        // Send powers to the wheels
        frontLeftDrive.power = frontLeftPower
        frontRightDrive.power = frontRightPower
        backLeftDrive.power = backLeftPower
        backRightDrive.power = backRightPower
    }

    /**
     * Initialize the AprilTag processor
     */
    private fun initAprilTag() {
        // Create the AprilTag processor
        aprilTag = AprilTagProcessor.Builder().build()

        // Adjust Image Decimation to trade-off detection-range for detection-rate
        aprilTag.setDecimation(2.0f)

        // Create the vision portal
        visionPortal = if (USE_WEBCAM) {
            VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
                .addProcessor(aprilTag)
                .build()
        } else {
            VisionPortal.Builder()
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(aprilTag)
                .build()
        }
    }

    /**
     * Update camera exposure (non-blocking state machine)
     * This replaces the blocking sleep() calls in the LinearOpMode version
     */
    private fun updateCameraExposure() {
        when (cameraState) {
            CameraState.WAITING -> {
                if (visionPortal.cameraState == VisionPortal.CameraState.STREAMING) {
                    // Camera is ready, set exposure
                    setManualExposure(6, 250)
                    cameraState = CameraState.READY
                }
            }
            else -> { /* Nothing to do */ }
        }
    }

    /**
     * Manually set the camera gain and exposure
     * This can only be called AFTER the camera is streaming, and only works for Webcams
     */
    private fun setManualExposure(exposureMS: Int, gain: Int) {
        val exposureControl = visionPortal.getCameraControl(ExposureControl::class.java)
        if (exposureControl.mode != ExposureControl.Mode.Manual) {
            exposureControl.mode = ExposureControl.Mode.Manual
        }
        exposureControl.setExposure(exposureMS.toLong(), TimeUnit.MILLISECONDS)

        val gainControl = visionPortal.getCameraControl(GainControl::class.java)
        gainControl.gain = gain
    }
}
