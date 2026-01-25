@file:Suppress("unused")

package org.beargineers.platform

import com.bylazar.field.PanelsField
import com.bylazar.panels.Panels
import com.bylazar.telemetry.PanelsTelemetry
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.PI
import kotlin.math.cosh
import kotlin.math.pow

fun cursorLocation(): Location{
    return Location(PanelsField.field.cursorX.inch,PanelsField.field.cursorY.inch)
}
abstract class BaseRobot(override val opMode: RobotOpMode<*>) : Robot {
    abstract val drive: Drivetrain

    abstract val localizer: Localizer
    val allHardware = mutableListOf<Hardware>()

    override val telemetry: Telemetry get() = opMode.telemetry

    val panelsTelemetry = PanelsTelemetry.telemetry
    val panelsField = PanelsField.field

    private val parts = mutableMapOf<Part<*>, Any>()

    override val currentPosition: Position get() = localizer.getPosition()

    override val currentVelocity: RelativePosition get() = localizer.getVelocity()
    override var targetSpeed: Double = 1.0


    override fun init() {
        allHardware.forEach {
            it.init()
        }
    }

    override fun assumePosition(position: Position) {
        localizer.setStartingPosition(position)
    }

    override fun isMoving(): Boolean {
        val vel = currentVelocity
        val lateral = vel.lateral().cm()
        val angular = vel.angular().degrees()

        return lateral > positionTolerance || angular > headingTolerance
    }

    @Suppress("UNCHECKED_CAST")
    override fun <T:Any> getPart(part: Part<T>): T {
        return parts.getOrPut(part) {
            part.build(this)
        } as T
    }

    override fun loop() {
        allHardware.forEach {
            it.loop()
        }

        localizer.update()

        telemetry.addData("Position", currentPosition)
        telemetry.addData("Velocity", "%s/s, %s/s",
            hypot(currentVelocity.forward, currentVelocity.right),
            abs(currentVelocity.turn))

        if (Panels.wasStarted) {
            drawRobotOnPanelsField()
            panelsTelemetry.update()
        }
    }

    private fun drawRobotOnPanelsField() {
        val cp = currentPosition
        with(panelsField) {
            fun lineTo(location: Location) {
                line(location.x.inch(), location.y.inch())
                moveCursor(location.x.inch(), location.y.inch())
            }
            fun moveCursor(location: Location) {
                moveCursor(location.x.inch(), location.y.inch())
            }

            setStyle("white", "blue", 1.0)
            val l = getPart(RobotLocations)
            moveCursor(l.lf_corner)
            lineTo(l.rf_corner)
            lineTo(l.rb_corner)
            lineTo(l.lb_corner)
            lineTo(l.lf_corner)

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
    override fun driveToTarget(target: Position): Boolean {
        return followPath(listOf(Waypoint(target)))
    }

    override fun driveByPowerAndAngle(theta: Double, power: Double, turn: Double) {
        drive.driveByPowerAndAngle(theta, power, turn)
    }

    override fun drive(
        forwardPower: Double,
        rightPower: Double,
        turnPower: Double
    ) {
        drive.drive(forwardPower, rightPower, turnPower)
    }

    override fun stopDriving() {
        drive.stop()
    }

    fun curveToTarget(target: Position, radius: Distance, clockwise: Boolean){
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

        driveToTarget(driveTo)
    }


    val minimalWheelPower by config(0.12)

    // Proportional control gains (tune these values for your robot).
    // Too low values will result in robot moving unnecessarily slow
    // Too high values will result in robot driving past destination and maybe even oscillating around it

    val drive_K by config(PIDFTCoeffs(0.025, 0.0, 0.00001, 0.6))
    val translational_K by config(PIDFTCoeffs(0.1, 0.0, 0.0, 0.015))
    val heading_K by config(PIDFTCoeffs(1.0, 0.0, 0.0, 0.01))

    val positionTolerance by config(2.0)
    val headingTolerance by config(5.0)

    val stalledPathAbortTimeoutMillis by config(100)
}