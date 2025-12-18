@file:Suppress("unused")

package org.beargineers.platform

import com.bylazar.field.PanelsField
import com.bylazar.telemetry.PanelsTelemetry
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.cosh
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sin

val positionTolerance: Double = 2.0
val headingTolerance: Double = 5.0


data class AutonomousDriveConfig(
    val minimalWheelPower: Double = 0.12,
    val maximumSpeed: Double = 1.0,

    // Proportional control gains (tune these values for your robot).
    // Too low values will result in robot moving unnecessarily slow
    // Too high values will result in robot driving past destination and maybe even oscillating around it


    val kP_position: Double = 0.035, // Position gain (power per distance unit)
    val kP_heading: Double = 0.01 // Heading gain (power per angle unit)
)

// Minimum confidence threshold to accept vision measurements
var MIN_VISION_CONFIDENCE = 0.3

abstract class BaseRobot(val opMode: RobotOpMode<*>) : Robot {
    abstract val drive: Drivetrain
    abstract val relativeLocalizer: RelativeLocalizer
    abstract val absoluteLocalizer: AbsoluteLocalizer

    // Kalman filter for sensor fusion
    abstract fun configureKalmanFilter(): KalmanFilter
    abstract fun configureAutonomousDriving(): AutonomousDriveConfig
    private val kalmanFilter = configureKalmanFilter()
    private val autoConfig = configureAutonomousDriving()

    val allHardware = mutableListOf<Hardware>()

    override val telemetry: Telemetry get() = opMode.telemetry

    val panelsTelemetry = PanelsTelemetry.telemetry
    val panelsField = PanelsField.field

    var lastTimeMovedNanos: Long = 0

    override var currentPosition: Position = FIELD_CENTER

    val currentVelocity: RelativePosition get() = relativeLocalizer.getVelocity()

    override fun init() {
        allHardware.forEach {
            it.init()
        }

        // Initialize Kalman filter with current position
        kalmanFilter.initialize(currentPosition)
    }

    override fun assumePosition(position: Position) {
        currentPosition = position
        kalmanFilter.initialize(position)
        relativeLocalizer.updatePositionEstimate(position)
    }

    override fun isMoving(): Boolean {
        val vel = currentVelocity
        return abs(vel.forward) + abs(vel.right) + abs(vel.turn) > 0.001
    }

    override fun loop() {
        allHardware.forEach {
            it.loop()
        }

        if (isMoving()) {
            lastTimeMovedNanos = System.nanoTime()
        }

        // Step 1: Prediction - update position using odometry (relative localizer)
        val basePosition = relativeLocalizer.getPosition()
        kalmanFilter.predictByAbsolute(basePosition)

        // Step 2: Correction - update position using vision (absolute localizer) if available
        val visionMeasurement = absoluteLocalizer.getRobotPose()
        if (visionMeasurement != null && visionMeasurement.timestampNano >= lastTimeMovedNanos && visionMeasurement.confidence >= MIN_VISION_CONFIDENCE) {
            kalmanFilter.correct(visionMeasurement)
            relativeLocalizer.updatePositionEstimate(kalmanFilter.getEstimate())
            telemetry.addData("Vision", "✓ conf=%.2f".format(visionMeasurement.confidence))
        } else {
            telemetry.addData("Vision", "✗ odometry only")
        }

        // Update current position from Kalman filter estimate
        currentPosition = kalmanFilter.getEstimate(DistanceUnit.CM, AngleUnit.RADIANS)

        // Display uncertainty for debugging
        val (xStd, yStd, headStd) = kalmanFilter.getUncertainty()
        telemetry.addData("Position", currentPosition)
        telemetry.addData("Uncertainty", "±%.1f cm, ±%.1f°".format(
            hypot(xStd, yStd),
            Math.toDegrees(headStd)
        ))
        telemetry.addData("Velocity", "%.2f cm/s", hypot(currentVelocity.forward, currentVelocity.right))

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

    override fun stop() {
        allHardware.forEach { it.stop() }
    }

    fun registerHardware(hardware: Hardware) {
        allHardware += hardware
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
    override fun driveToTarget(target: Position, maxSpeed: Double): Boolean {
        // Make sure current and target positions are in the same units.
        val cp = currentPosition.toDistanceUnit(DistanceUnit.CM).toAngleUnit(AngleUnit.RADIANS)
        val tp = target.toDistanceUnit(DistanceUnit.CM).toAngleUnit(AngleUnit.RADIANS)

        // Calculate position error (field frame)
        val deltaX = tp.x - cp.x
        val deltaY = tp.y - cp.y

        // Calculate distance to target
        val distanceToTarget = hypot(deltaX, deltaY)

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
        val forwardPower = robotForward * autoConfig.kP_position
        val strafePower = robotRight * autoConfig.kP_position
        val turnPower = -Math.toDegrees(headingError) * autoConfig.kP_heading // Positive power to turn CW, but positive delta heating is CCW

        val maxPower = listOf(forwardPower, strafePower, turnPower).maxOf { abs(it) }
        val maxV = when {
            maxPower > maxSpeed -> maxPower / maxSpeed
            maxPower < autoConfig.minimalWheelPower -> maxPower / autoConfig.minimalWheelPower
            else -> 1.0
        }

        fun clamp(value: Double): Double {
            return value / maxV
        }

        // Apply drive power
        drive(
            forwardPower = clamp(forwardPower),
            rightPower = clamp(strafePower),
            turnPower = clamp(turnPower)
        )

        return true
    }

    override fun drive(
        forwardPower: Double,
        rightPower: Double,
        turnPower: Double,
        slow: Boolean
    ) {
        drive.drive(forwardPower, rightPower, turnPower, slow)
    }

    override fun stopDriving() {
        drive.stop()
    }

    fun curveToTarget(target: Position, radius: Double, CW: Boolean, radiusDistanceUnit: DistanceUnit , maxSpeed: Double){

        val r = target.distanceUnit.fromUnit(radiusDistanceUnit, radius)

        val cp = currentPosition.toDistanceUnit(target.distanceUnit)

        val distanceToTarget: Double = hypot((target.x - cp.x), (target.y - cp.y))

        val t = cosh(1- 0.5*(distanceToTarget/r).pow(2))

        val t2: Double = PI/4 - 0.5 * t

        val curvedDistanceToTarget: Double = t * r

        val vectorHeading: Double = when(CW){
            true -> atan2((target.y - cp.y), (target.x - cp.x)) + (PI/4 - t2)
            false -> atan2((target.y - cp.y), (target.x - cp.x)) - (PI/4 - t2)
        }

        val driveTo = Position(curvedDistanceToTarget * cos(vectorHeading), curvedDistanceToTarget * sin(vectorHeading), target.heading, target.distanceUnit, target.angleUnit)

        driveToTarget(driveTo, maxSpeed)

    }
}