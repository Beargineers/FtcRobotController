package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.field.PanelsField
import com.bylazar.telemetry.PanelsTelemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.internal.Hardware
import org.firstinspires.ftc.teamcode.internal.RobotOpModeBase
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

@Configurable
object AutonomousConfig {
    var MAX_SPEED = 0.7

    // Proportional control gains (tune these values for your robot)
    var kP_position = 0.035  // Position gain (power per distance unit)
    var kP_heading = 0.01   // Heading gain (power per angle unit)
}

abstract class Robot(val alliance: Alliance) : RobotOpModeBase() {
    lateinit var drive: Drivebase
    lateinit var aprilTags: AprilTagWebcam
    lateinit var shooter: Shooter
    lateinit var intake: Intake

    val allHardware = mutableListOf<Hardware>()

    val panelsTelemetry = PanelsTelemetry.telemetry
    val panelsField = PanelsField.field

    var currentPose: Pose2D = FIELD_CENTER.withHeading(0.0, AngleUnit.DEGREES)
    var goalDistanceCM: Double? = null

    var atChange: Pose2D = Pose2D(0.0, 0.0, 0.0)
    var dbChange: Pose2D = Pose2D(0.0, 0.0, 0.0)

    override fun init() {
        drive = Drivebase(this)
        aprilTags = AprilTagWebcam(this)
        shooter = Shooter(this)
        intake = Intake(this)

        allHardware += drive
        allHardware += aprilTags
        allHardware += shooter
        allHardware += intake

        allHardware.forEach {
            it.init()
        }

        telemetry.addLine("Ready")
    }

    override fun loop() {
        allHardware.forEach { it.loop() }
        val driveChange = drive.poseChange(currentPose.toAngleUnit(AngleUnit.RADIANS).heading)

        currentPose += driveChange
        if (abs(driveChange.x) > 0.1 ||
            abs(driveChange.y) > 0.1 ||
            abs(driveChange.heading) > 0.1) {
            goalDistanceCM = null
        }

        val atRange = aprilTags.findTarget(alliance)?.ftcPose?.range
        if (atRange != null) {
            goalDistanceCM = atRange
        }

        telemetry.addData("Pose", currentPose)
        telemetry.addData("Goal distance", goalDistanceCM ?: "goal not found")
        telemetry.update()

        drawRobotOnPanelsField()
    }

    private fun drawRobotOnPanelsField() {
        val cp = currentPose.toDistanceUnit(DistanceUnit.INCH).toAngleUnit(AngleUnit.RADIANS)

        panelsField.moveCursor(cp.x, cp.y)
        panelsField.circle(2.0)

        panelsField.moveCursor(cp.x + 3 * cos(cp.heading), cp.y + 3 * sin(cp.heading))
        panelsField.circle(1.0)
        panelsField.update()
    }

    /**
     * Continuously drives the robot toward a target pose using proportional control.
     *
     * This function calculates the error between the current pose and target pose,
     * then applies proportional power to the mecanum drive motors to reduce that error.
     * This function should be called repeatedly in a loop until the robot reaches the target.
     *
     * The function uses simple proportional (P) control:
     * - Position error is converted to drive power (forward/strafe)
     * - Heading error is converted to rotational power
     * - Power is clamped to safe limits
     *
     * @param pose Target pose (position and heading) to drive to
     * @param positionTolerance Distance threshold in the pose's distance units (default 2.0)
     * @param headingTolerance Angle threshold in the pose's angle units (default 5.0)
     * @return true if robot has reached the target within tolerances, false otherwise
     *
     * ## Example Usage
     * ```kotlin
     * val targetPose = Pose2D(100.0, 50.0, 90.0, DistanceUnit.CM, AngleUnit.DEGREES)
     *
     * override fun loop() {
     *     super.loop()
     *     if (!driveToPose(targetPose)) {
     *         telemetry.addLine("Driving to target...")
     *     } else {
     *         telemetry.addLine("Target reached!")
     *         drive.stop()
     *     }
     * }
     * ```
     */
    fun driveToPose(
        pose: Pose2D,
        maxSpeed: Double,
        positionTolerance: Double = 2.0,
        headingTolerance: Double = 5.0
    ): Boolean {
        val maxSpeed = min(maxSpeed, AutonomousConfig.MAX_SPEED)
        // Make sure current and target positions are in the same units.
        val cp = currentPose.toDistanceUnit(DistanceUnit.CM).toAngleUnit(AngleUnit.RADIANS)
        val tp = pose.toDistanceUnit(DistanceUnit.CM).toAngleUnit(AngleUnit.RADIANS)

        // Calculate position error (field frame)
        val deltaX = tp.x - cp.x
        val deltaY = tp.y - cp.y

        // Calculate distance to target
        val distanceToTarget = sqrt(deltaX.pow(2) + deltaY.pow(2))

        // Calculate heading error (normalized to -180 to 180 degrees equivalent)
        var headingError = tp.heading - cp.heading
        val halfCircle = PI
        val fullCircle = halfCircle * 2
        // Normalize heading error to [-180, 180] or [-π, π]
        while (headingError > halfCircle) headingError -= fullCircle
        while (headingError < -halfCircle) headingError += fullCircle

        telemetry.addData("Distance to target", "%.2f %s".format(distanceToTarget, tp.distanceUnit))
        telemetry.addData("Heading Error", headingError)


        // Check if we've reached the target
        if (distanceToTarget < positionTolerance && Math.toDegrees(abs(headingError)) < headingTolerance) {
            drive.stop()
            return false
        }

        // Convert field-frame error to robot-frame error
        // We need to rotate the field-frame delta by the negative of the robot's heading
        val robotHeadingRad = cp.heading
        val robotForward = deltaX * cos(robotHeadingRad) + deltaY * sin(robotHeadingRad)
        val robotRight = deltaX * sin(robotHeadingRad) - deltaY * cos(robotHeadingRad)

        // Calculate drive powers using proportional control
        val forwardPower = robotForward * AutonomousConfig.kP_position
        val strafePower = robotRight * AutonomousConfig.kP_position
        val turnPower = -Math.toDegrees(headingError) * AutonomousConfig.kP_heading // Positive power to turn CW, but positive delta heating is CCW

        val maxPower = listOf(forwardPower, strafePower, turnPower).maxOf { abs(it) }
        val maxV = when {
            maxPower > maxSpeed -> maxPower / maxSpeed
            maxPower < 0.08 -> maxPower / 0.08
            else -> 1.0
        }

        fun clamp(value: Double): Double {
            return value / maxV
        }

        // Apply drive power
        drive.drive(
            forward = clamp(forwardPower),
            right = clamp(strafePower),
            turn = clamp(turnPower)
        )

        // Add telemetry for debugging
        telemetry.addData("Distance to target", "%.2f %s".format(distanceToTarget, tp.distanceUnit))
        telemetry.addData("Heading error", "%.2f %s".format(headingError, tp.angleUnit))
        telemetry.addData("Drive powers (y,x,turn)", "%.2f, %.2f, %.2f".format(
            clamp(forwardPower), clamp(strafePower), clamp(turnPower)
        ))

        return true
    }

    override fun stop() {
        allHardware.forEach { it.stop() }
    }

    fun resetCoords() {
        atChange = Pose2D(0.0, 0.0, 0.0)
        dbChange = Pose2D(0.0, 0.0, 0.0)
    }
}