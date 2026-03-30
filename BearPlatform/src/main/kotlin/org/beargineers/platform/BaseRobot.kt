@file:Suppress("unused")

package org.beargineers.platform

import com.bylazar.field.PanelsField
import com.bylazar.panels.Panels
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.rr.RRMecanumDrive

fun cursorLocation(): Location{
    return Location(PanelsField.field.cursorX.inch,PanelsField.field.cursorY.inch)
}

private val locationHistorySize by config(50)
private val PANELS_FIELD_FPS by config(3)
private val PANELS_SHOW_PATH by config(false)

abstract class BaseRobot(override val opMode: RobotOpMode<*>) : Robot {
    val allHardware = mutableListOf<Hardware>()
    val drive = MecanumDrive(this)
    val rrMecanumDrive by lazy { RRMecanumDrive(this) }

    abstract val localizer: Localizer

    val panelsTelemetry = PanelsTelemetry.telemetry
    val panelsField = PanelsField.field

    private val parts = mutableMapOf<Part<*>, Any>()

    val states = mutableMapOf<StateHolder<*>, Any>()

    override val currentPosition: Position get() = localizer.getPosition()

    override val currentVelocity: Position get() = localizer.getVelocity()

    private val locationHistory = ArrayDeque<Location>(locationHistorySize)

    private var hasBeenInitialized = false

    override fun init() {
        allHardware.forEach {
            it.init()
        }
        hasBeenInitialized = true
    }

    override fun start() {
    }

    override fun assumePosition(position: Position, turretAngle: Angle) {
        localizer.setStartingPosition(position)
    }

    override fun predictedPosition(nTicks: Int): Position {
        val durationSec = nTicks * opMode.normalTickDurationMs() / 1000
        return currentPosition + currentVelocity * durationSec
    }

    fun isMoving(): Boolean {
        val vel = currentVelocity
        val lateral = hypot(vel.x, vel.y)
        val angular = abs(vel.heading.normalize())

        return lateral > PathFollowingConfig.positionToleranceToStop || angular > PathFollowingConfig.headingToleranceToStop
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

        Frame.addData("Position", currentPosition)
        Frame.addData("Velocity", "%s/s, %s/s",
            hypot(currentVelocity.x, currentVelocity.y),
            abs(currentVelocity.heading))

        for ((key, value) in states) {
            Frame.addData(key.name, value)
        }

        locationHistory.addLast(currentPosition.location())
        while (locationHistory.size >= locationHistorySize) {
            locationHistory.removeFirst()
        }

        if (Panels.wasStarted) {
            drawRobotOnPanelsField()
            panelsTelemetry.update()
        }
    }

    private val drawnAt = ElapsedTime()
    private fun drawRobotOnPanelsField() {
        if (drawnAt.milliseconds() > 1000 / PANELS_FIELD_FPS) {
            drawnAt.reset()

            doDrawRobot()
        }
    }

    private fun doDrawRobot() {
        if (Panels.clientsCount == 0) return

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

            if (PANELS_SHOW_PATH) {
                setStyle("", "#4CAF50", 0.75)

                for ((prev, next) in locationHistory.zipWithNext()) {
                    moveCursor(prev.x.inch(), prev.y.inch())
                    line(next.x.inch(), next.y.inch())
                }
            }

            update()
        }
    }

    override fun stop() {
        allHardware.forEach { it.stop() }
    }

    fun registerHardware(hardware: Hardware) {
        allHardware += hardware
        if (hasBeenInitialized) {
            hardware.init()
        }
    }

    fun motorPowers(
        forwardPower: Double,
        rightPower: Double,
        turnPower: Double
    ) {
        drive.drive(forwardPower, rightPower, turnPower)
    }

    fun stopDriving() {
        drive.stop()
    }

    val minimalWheelPower by config(0.12)

    // Proportional control gains (tune these values for your robot).
    // Too low values will result in robot moving unnecessarily slow
    // Too high values will result in robot driving past destination and maybe even oscillating around it

    val drive_K by config(PIDFTCoeffs(0.025, 0.0, 0.00001, 0.6))
    val drive_K2 by config(PIDFTCoeffs(0.0, 0.0, 0.0, 0.0))
    val translational_K by config(PIDFTCoeffs(0.1, 0.0, 0.0, 0.015))
    val heading_K by config(PIDFTCoeffs(1.0, 0.0, 0.0, 0.01))
}