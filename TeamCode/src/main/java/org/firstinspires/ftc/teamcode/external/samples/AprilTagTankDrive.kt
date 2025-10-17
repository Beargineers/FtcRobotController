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
const val TANK_DESIRED_DISTANCE = 12.0  // How close the camera should get to the target (inches)
const val TANK_SPEED_GAIN = 0.02        // Forward speed control gain
const val TANK_TURN_GAIN = 0.01         // Turn control gain
const val TANK_MAX_AUTO_SPEED = 0.5     // Clip the approach speed to this max value
const val TANK_MAX_AUTO_TURN = 0.25     // Clip the turn speed to this max value
const val TANK_USE_WEBCAM = true        // Set true to use a webcam, or false for a phone camera
const val TANK_DESIRED_TAG_ID = -1      // Choose the tag you want to approach or set to -1 for ANY tag

/**
 * Tank Drive To AprilTag - Kotlin Iterative Implementation
 *
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a basic two-wheel (Tank) Robot Drivetrain.
 *
 * The driving goal is to rotate to keep the tag centered in the camera, while driving
 * towards the tag to achieve the desired distance.
 *
 * Controls:
 * - Left stick Y: Forward/back (manual mode)
 * - Right stick X: Rotation (manual mode)
 * - Left bumper: Hold to enable automatic "Drive to target" mode
 *
 * Under "Drive To Target" mode, the robot has two goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame
 * 2) Drive towards the Tag to get to the desired distance
 */
@TeleOp(name = "Tank Drive To AprilTag", group = "Concept")
@Disabled
class AprilTagTankDrive : RobotOpModeBase() {
    // Motors
    val leftDrive: DcMotor by hardware("left_drive")
    val rightDrive: DcMotor by hardware("right_drive")

    // Vision
    private lateinit var visionPortal: VisionPortal
    private lateinit var aprilTag: AprilTagProcessor

    // State for camera initialization
    private enum class CameraState { NOT_STARTED, WAITING, READY }
    private var cameraState = CameraState.NOT_STARTED

    override fun init() {
        // Configure motor directions
        leftDrive.direction = DcMotorSimple.Direction.REVERSE
        rightDrive.direction = DcMotorSimple.Direction.FORWARD

        // Initialize AprilTag detection
        initAprilTag()

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream")
        telemetry.addData(">", "Robot Ready. Press START.")
        telemetry.update()
    }

    override fun start() {
        super.start()
        // Start camera exposure configuration
        if (TANK_USE_WEBCAM) {
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
                if ((TANK_DESIRED_TAG_ID < 0) || (detection.id == TANK_DESIRED_TAG_ID)) {
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
            }
        } else {
            telemetry.addData("\n>", "Drive using joysticks to find valid target\n")
        }

        // Calculate drive and turn values
        val drive: Double
        val turn: Double

        if (gamepad1.left_bumper && targetFound && desiredTag != null) {
            // Automatic mode - drive to target
            val rangeError = desiredTag.ftcPose.range - TANK_DESIRED_DISTANCE
            val headingError = desiredTag.ftcPose.bearing

            drive = Range.clip(rangeError * TANK_SPEED_GAIN, -TANK_MAX_AUTO_SPEED, TANK_MAX_AUTO_SPEED)
            turn = Range.clip(headingError * TANK_TURN_GAIN, -TANK_MAX_AUTO_TURN, TANK_MAX_AUTO_TURN)

            telemetry.addData("Auto", "Drive %5.2f, Turn %5.2f", drive, turn)
        } else {
            // Manual mode - use joysticks (POV mode)
            drive = -gamepad1.left_stick_y.toDouble() / 2.0   // Reduce drive rate to 50%
            turn = -gamepad1.right_stick_x.toDouble() / 4.0   // Reduce turn rate to 25%
            telemetry.addData("Manual", "Drive %5.2f, Turn %5.2f", drive, turn)
        }

        telemetry.update()

        // Apply desired axes motions to the drivetrain
        moveRobot(drive, turn)
    }

    override fun stop() {
        visionPortal.close()
    }

    /**
     * Move robot according to desired axes motions
     * Positive X is forward
     * Positive Yaw is counter-clockwise
     */
    private fun moveRobot(x: Double, yaw: Double) {
        // Calculate left and right wheel powers
        var leftPower = x - yaw
        var rightPower = x + yaw

        // Normalize wheel powers to be less than 1.0
        val max = maxOf(Math.abs(leftPower), Math.abs(rightPower))
        if (max > 1.0) {
            leftPower /= max
            rightPower /= max
        }

        // Send powers to the wheels
        leftDrive.power = leftPower
        rightDrive.power = rightPower
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
        visionPortal = if (TANK_USE_WEBCAM) {
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
