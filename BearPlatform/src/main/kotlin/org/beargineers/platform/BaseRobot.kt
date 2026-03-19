@file:Suppress("unused")

package org.beargineers.platform

import com.bylazar.field.PanelsField
import com.bylazar.panels.Panels
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.rr.MecanumDrive
import org.firstinspires.ftc.robotcore.external.Telemetry

fun cursorLocation(): Location{
    return Location(PanelsField.field.cursorX.inch,PanelsField.field.cursorY.inch)
}

private val locationHistorySize by config(50)
private val PANELS_FIELD_FPS by config(3)
private val PANELS_SHOW_PATH by config(false)

abstract class BaseRobot(override val opMode: RobotOpMode<*>) : Robot {
    abstract val drive: Drivetrain

    val mecanumDrive by lazy {  MecanumDrive(this) }

    abstract val localizer: Localizer
    val allHardware = mutableListOf<Hardware>()

    override val telemetry: Telemetry get() = opMode.telemetry

    val panelsTelemetry = PanelsTelemetry.telemetry
    val panelsField = PanelsField.field

    private val parts = mutableMapOf<Part<*>, Any>()

    override val currentPosition: Position get() = localizer.getPosition()

    override val currentVelocity: RobotCentricPosition get() = localizer.getVelocity()
    override var targetSpeed: Double = 1.0

    private val locationHistory = ArrayDeque<Location>(locationHistorySize)


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
        val lateral = abs(vel.linear())
        val angular = abs(vel.angular().normalize())

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
            try {
                it.loop()
            } catch (e: Exception) {
                e.printStackTrace()
            }
        }

        localizer.update()

        telemetry.addData("Position", currentPosition)
        telemetry.addData("Velocity", "%s/s, %s/s",
            hypot(currentVelocity.forward, currentVelocity.right),
            abs(currentVelocity.turn))

        locationHistory.addLast(currentPosition.location())
        while (locationHistory.size >= locationHistorySize) {
            locationHistory.removeFirst()
        }

        panelsTelemetry.addData("Position", currentPosition)

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
    }


    override fun motorPowers(
        forwardPower: Double,
        rightPower: Double,
        turnPower: Double
    ) {
        drive.drive(forwardPower, rightPower, turnPower)
    }

    override fun stopDriving() {
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