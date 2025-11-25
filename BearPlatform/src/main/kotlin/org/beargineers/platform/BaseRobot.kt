@file:Suppress("unused")

package org.beargineers.platform

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.field.PanelsField
import com.bylazar.telemetry.PanelsTelemetry
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt


@Configurable
object AutonomousConfig {
    var MINIMAL_WHEEL_POWER: Double = 0.12

    // Max default auto speed. Pass maxSpeed parameter to driveTo to overrun the default
    var MAX_SPEED = 0.7


    // Proportional control gains (tune these values for your robot)
    var kP_position = 0.035  // Position gain (power per distance unit)
    var kP_heading = 0.01   // Heading gain (power per angle unit)
}

abstract class BaseRobot(val opMode: RobotOpMode<*>) {
    abstract val drive: Drivetrain
    val allHardware = mutableListOf<Hardware>()

    val telemetry: Telemetry get() = opMode.telemetry

    val panelsTelemetry = PanelsTelemetry.telemetry
    val panelsField = PanelsField.field

    var currentPosition: Position = FIELD_CENTER
    var currentMove: RobotMovement = RobotMovement.zero()

    open fun init() {
        allHardware.forEach {
            it.init()
        }
    }

    open fun loop() {
        allHardware.forEach {
            it.loop()
        }

        val move = drive.robotMovedBy().toUnits(DistanceUnit.CM, AngleUnit.RADIANS)
        currentMove = move
        currentPosition += positionChange(move)

        telemetry.addData("Position", currentPosition)
        telemetry.addData("Moved", move)

        drawRobotOnPanelsField()
    }

    private fun drawRobotOnPanelsField() {
        val cp = currentPosition.toDistanceUnit(DistanceUnit.INCH).toAngleUnit(AngleUnit.RADIANS)

        panelsField.moveCursor(cp.x, cp.y)
        panelsField.circle(2.0)

        panelsField.moveCursor(cp.x + 3 * cos(cp.heading), cp.y + 3 * sin(cp.heading))
        panelsField.circle(1.0)
        panelsField.update()
    }

    open fun stop() {
        allHardware.forEach { it.stop() }
    }

    fun registerHardware(hardware: Hardware) {
        allHardware += hardware
    }

    /**
     * Calculates the change in robot pose based on encoder readings.
     *
     * Uses mecanum wheel forward kinematics to determine the robot's movement
     * in the robot frame since the last encoder reset.
     *
     * Forward kinematics for mecanum drive:
     * - Forward/backward (y): All wheels contribute equally
     * - Strafe left/right (x): Diagonal wheels oppose each other
     * - Heading: From IMU yaw angle
     *
     * @return Pose2D representing the change in position and orientation
     */
    private fun positionChange(move: RobotMovement): Position {
        val (forward, right, deltaYaw) = move

        val N = 10
        val fn = forward / N
        val rn = right / N

        var deltaX = 0.0
        var deltaY = 0.0

        val oldHeading: Double = currentPosition.toAngleUnit(AngleUnit.RADIANS).heading
        var heading = oldHeading

        repeat(N) {
            deltaX += fn * cos(heading) + rn * sin(heading)
            deltaY += fn * sin(heading) - rn * cos(heading)

            heading += deltaYaw / N
        }

        return Position(
            // Forward: all wheels contribute equally
            x = deltaX,
            // Strafe: diagonal wheels oppose (LF and RB forward = strafe right)
            y = deltaY,
            // Get yaw in radians to match the angleUnit specification
            heading = deltaYaw,
            distanceUnit = DistanceUnit.CM,
            angleUnit = AngleUnit.RADIANS
        ).normalizeHeading()
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
     * @param target Target pose (position and heading) to drive to
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
    fun driveToTarget(
        target: Position,
        maxSpeed: Double,
        positionTolerance: Double = 2.0,
        headingTolerance: Double = 5.0
    ): Boolean {
        val maxSpeed = min(maxSpeed, AutonomousConfig.MAX_SPEED)
        // Make sure current and target positions are in the same units.
        val cp = currentPosition.toDistanceUnit(DistanceUnit.CM).toAngleUnit(AngleUnit.RADIANS)
        val tp = target.toDistanceUnit(DistanceUnit.CM).toAngleUnit(AngleUnit.RADIANS)

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
            maxPower < AutonomousConfig.MINIMAL_WHEEL_POWER -> maxPower / AutonomousConfig.MINIMAL_WHEEL_POWER
            else -> 1.0
        }

        fun clamp(value: Double): Double {
            return value / maxV
        }

        // Apply drive power
        drive.drive(
            forwardPower = clamp(forwardPower),
            rightPower = clamp(strafePower),
            turnPower = clamp(turnPower)
        )

        return true
    }
}