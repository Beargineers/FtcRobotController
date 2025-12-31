@file:Suppress("unused")

package org.beargineers.platform

import com.bylazar.field.PanelsField
import com.bylazar.telemetry.PanelsTelemetry
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.util.Properties
import kotlin.math.PI
import kotlin.math.cosh
import kotlin.math.pow

abstract class BaseRobot(override val opMode: RobotOpMode<*>) : Robot {
    abstract val drive: Drivetrain
    abstract val relativeLocalizer: RelativeLocalizer
    abstract val absoluteLocalizer: AbsoluteLocalizer

    abstract val configResource: Int

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
        return Properties().apply {
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
        SettingsWebServer.initRobot(this)
        allHardware.forEach {
            it.init()
        }
    }

    override fun assumePosition(position: Position) {
        currentPosition = position
        relativeLocalizer.updatePositionEstimate(position)
    }

    override fun isMoving(): Boolean {
        val vel = currentVelocity
        return abs(vel.forward).cm() + abs(vel.right).cm() + abs(vel.turn).degrees() > 0.1
    }

    override fun loop() {
        allHardware.forEach {
            it.loop()
        }

        if (isMoving()) {
            lastTimeMovedNanos = System.nanoTime()
        }

        // Once we get reliable vision result, use it for current position and also update odometry computer
        val visionMeasurement = absoluteLocalizer.getRobotPose()
        if (visionMeasurement != null) {
            telemetry.addData("Vision", "✓ acquired")
            relativeLocalizer.updatePositionEstimate(visionMeasurement)
            currentPosition = visionMeasurement
        } else {
            telemetry.addData("Vision", "✗ odometry only")
            currentPosition = relativeLocalizer.getPosition()
        }

        telemetry.addData("Position", currentPosition)
        telemetry.addData("Velocity", "%s/s", hypot(currentVelocity.forward, currentVelocity.right))

        drawRobotOnPanelsField()
    }

    private fun drawRobotOnPanelsField() {
        val cp = currentPosition
        with(panelsField) {
            fun lineTo(location: Location) {
                line(location.x.inch(), location.y.inch())
                moveCursor(location.x.inch(), location.y.inch())
            }

            val xFromCenter = 9.inch // TODO: Real robot dimensions
            val yFromCenter = 9.inch
            val fr = Location(xFromCenter,yFromCenter).toAbsolute(cp)
            val fl = Location(-xFromCenter,yFromCenter).toAbsolute(cp)
            val br = Location(xFromCenter,-yFromCenter).toAbsolute(cp)
            val bl = Location(-xFromCenter,-yFromCenter).toAbsolute(cp)

            setStyle("white", "blue", 1.0)
            moveCursor(fl.x.inch(), fl.y.inch())
            lineTo(fr)
            lineTo(br)
            lineTo(bl)
            lineTo(fl)

            setStyle("white", "white", 1.0)
            moveCursor(cp.x.inch(), cp.y.inch())
            circle(2.0)

            moveCursor(cp.x.inch() + 3 * cos(cp.heading), cp.y.inch() + 3 * sin(cp.heading))
            circle(1.0)

            update()
        }
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
        return followPath(listOf(target), maxSpeed)
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



    override fun followPath(
        path: List<Position>,
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