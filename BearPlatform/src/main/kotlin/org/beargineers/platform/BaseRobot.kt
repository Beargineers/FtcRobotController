@file:Suppress("unused")

package org.beargineers.platform

import com.bylazar.field.PanelsField
import com.bylazar.telemetry.PanelsTelemetry
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.util.Properties
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cosh
import kotlin.math.hypot
import kotlin.math.pow

abstract class BaseRobot(override val opMode: RobotOpMode<*>) : Robot {
    abstract val drive: Drivetrain
    abstract val relativeLocalizer: RelativeLocalizer
    abstract val absoluteLocalizer: AbsoluteLocalizer

    abstract val configResource: Int

    // Kalman filter for sensor fusion
    private val kalmanFilter = KalmanFilter(this)
    val allHardware = mutableListOf<Hardware>()

    override val telemetry: Telemetry get() = opMode.telemetry

    val panelsTelemetry = PanelsTelemetry.telemetry
    val panelsField = PanelsField.field

    var lastTimeMovedNanos: Long = 0

    override var currentPosition: Position = FIELD_CENTER

    val currentVelocity: RelativePosition get() = relativeLocalizer.getVelocity()

    internal var currentConfigText = ""
    private set

    private var configs = Properties()

    private fun readConfigs(): Properties {
        val defaults = Properties().apply {
            load(opMode.hardwareMap.appContext.resources.openRawResource(R.raw.config))
        }

        return Properties(defaults).apply {
            load(currentConfigText.reader())
        }
    }

    fun updateConfigText(text: String) {
        currentConfigText = text
        configs = readConfigs()
    }

    override fun configValue(name: String): String? {
        return configs[name] as? String
    }

    override fun init() {
        updateConfigText(opMode.hardwareMap.appContext.resources.openRawResource(configResource).reader().readText())
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
        return abs(vel.forward).cm() + abs(vel.right).cm() + abs(vel.turn).degrees() > 0.001
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
        currentPosition = kalmanFilter.getEstimate()

        // Display uncertainty for debugging
        val (xStd, yStd, headStd) = kalmanFilter.getUncertainty()
        telemetry.addData("Position", currentPosition)
        telemetry.addData("Uncertainty", "±%.1f cm, ±%.1f°".format(
            hypot(xStd, yStd),
            Math.toDegrees(headStd)
        ))
        telemetry.addData("Velocity", "%s/s", hypot(currentVelocity.forward, currentVelocity.right))

        drawRobotOnPanelsField()
    }

    private fun drawRobotOnPanelsField() {
        val cp = currentPosition
        panelsField.moveCursor(cp.x.inch(), cp.y.inch())
        panelsField.circle(2.0)

        panelsField.moveCursor(cp.x.inch() + 3 * cos(cp.heading), cp.y.inch() + 3 * sin(cp.heading))
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
     * @return false if robot has reached the target within tolerances, true otherwise
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
        val cp = currentPosition
        val tp = target

        // Calculate position error (field frame)
        val deltaX = tp.x - cp.x
        val deltaY = tp.y - cp.y

        // Calculate distance to target
        val distanceToTarget = hypot(deltaX, deltaY)

        // Calculate heading error (normalized to -180 to 180 degrees equivalent)
        val headingError = (tp.heading - cp.heading).normalize()

        telemetry.addData("Distance to target", distanceToTarget)
        telemetry.addData("Heading Error", headingError)

        // Check if we've reached the target
        if (distanceToTarget.cm() < positionTolerance && abs(headingError).degrees() < headingTolerance) {
            drive.stop()
            return false
        }

        // Convert field-frame error to robot-frame error
        // We need to rotate the field-frame delta by the negative of the robot's heading
        val robotHeadingRad = cp.heading
        val robotForward = deltaX * cos(robotHeadingRad) + deltaY * sin(robotHeadingRad)
        val robotRight = deltaX * sin(robotHeadingRad) - deltaY * cos(robotHeadingRad)

        // Calculate drive powers using proportional control
        val forwardPower = robotForward.cm() * kP_position
        val strafePower = robotRight.cm() * kP_position
        val turnPower = -headingError.degrees() * kP_heading // Positive power to turn CW, but positive delta heating is CCW

        val maxPower = listOf(forwardPower, strafePower, turnPower).maxOf { abs(it) }
        val maxV = when {
            maxPower > maxSpeed -> maxPower / maxSpeed
            maxPower < minimalWheelPower -> maxPower / minimalWheelPower
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

    fun curveToTarget(target: Position, radius: Distance, clockwise: Boolean, maxSpeed: Double){
        val r = radius
        val cp = currentPosition

        val distanceToTarget = hypot((target.x - cp.x), (target.y - cp.y))
        val t = cosh(1- 0.5*(distanceToTarget/r).pow(2))
        val t2: Double = PI/4 - 0.5 * t
        val curvedDistanceToTarget = t * r

        val vectorHeading = when(clockwise){
            true -> atan2((target.y - cp.y), (target.x - cp.x)) + ((PI/4).radians - t2.radians)
            false -> atan2((target.y - cp.y), (target.x - cp.x)) - ((PI/4).radians - t2.radians)
        }

        val driveTo = Position(
            curvedDistanceToTarget * cos(vectorHeading),
            curvedDistanceToTarget * sin(vectorHeading),
            target.heading)

        driveToTarget(driveTo, maxSpeed)
    }


    val minimalWheelPower by config(0.12)

    // Proportional control gains (tune these values for your robot).
    // Too low values will result in robot moving unnecessarily slow
    // Too high values will result in robot driving past destination and maybe even oscillating around it

    val kP_position by config(0.035) // Position gain (power per distance unit)
    val kP_heading by config(0.01) // Heading gain (power per angle unit)

    val positionTolerance by config(2.0)
    val headingTolerance by config(5.0)


    // Minimum confidence threshold to accept vision measurements
    val MIN_VISION_CONFIDENCE by config(0.3)

    override fun followPath(
        path: Path,
        maxSpeed: Double
    ): Boolean {
        // Check if this is a new path (different instance)
        if (currentFollower?.path != path) {
            // Create new PathFollower for this path
            currentFollower = PathFollower(
                path = path,
                robot = this,
                startPosition = currentPosition,
                maxSpeed = maxSpeed,
                kP_position = kP_position,
                kP_heading = kP_heading,
                minimalWheelPower = minimalWheelPower,
                positionTolerance = positionTolerance,
                headingTolerance = headingTolerance
            )
        }

        return currentFollower!!.update()
    }

    override fun stopFollowingPath() {
        currentFollower = null
        stopDriving()
    }


    // Track which path we're currently following (by reference)
    private var currentFollower: PathFollower? = null

}