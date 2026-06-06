package org.beargineers.platform

import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoControllerEx
import kotlin.time.Duration

enum class LEDColor(val pos: Double) {
    OFF(0.2),
    RED(0.277),
    ORANGE(0.333),
    YELLOW(0.388),
    SAGE(0.444),
    GREEN(0.5),
    AZURE(0.555),
    BLUE(0.611),
    INDIGO(0.666),
    VIOLET(0.722),
    WHITE(1.0)
}

interface RGBSignal {
    val priority: Int
}

class RGBIndicator(robot: BaseRobot): Hardware(robot) {
    val led by hardware<Servo>()

    private sealed class Mode {
        data object Glow : Mode()
        data class Blink(val times: Int, val periodMs: Long) : Mode()
    }

    private data class Request(
        val color: LEDColor,
        val priority: Int,
        val mode: Mode,
        val startedAtMs: Long,
        val revision: Long
    )

    private val requests = mutableMapOf<RGBSignal, Request>()
    private var revision = 0L
    private var displayedColor: LEDColor? = null

    override fun init() {
        (led.controller as ServoControllerEx).setServoPwmRange(
            led.portNumber,
            PwmControl.PwmRange(500.0, 2500.0)
        )
        setColor(LEDColor.OFF)
    }

    override fun loop() {
        val now = System.currentTimeMillis()
        requests.entries.removeAll { (_, request) -> request.isFinished(now) }
        setColor(resolveColor(now))
    }

    fun glow(key: RGBSignal, color: LEDColor) {
        setRequest(key, color, Mode.Glow)
    }

    fun blink(key: RGBSignal, color: LEDColor, times: Int, period: Duration) {
        require(times > 0) { "Blink times must be positive" }
        val periodMs = period.inWholeMilliseconds
        require(periodMs > 0) { "Blink period must be positive" }
        setRequest(key, color, Mode.Blink(times, periodMs))
    }

    fun clear(key: RGBSignal) {
        requests.remove(key)
    }

    private fun setRequest(key: RGBSignal, color: LEDColor, mode: Mode) {
        val previous = requests[key]
        if (previous?.color == color && previous.priority == key.priority && previous.mode == mode) {
            return
        }

        requests[key] = Request(color, key.priority, mode, System.currentTimeMillis(), revision++)
    }

    private fun resolveColor(now: Long): LEDColor {
        val request = requests.values.maxWithOrNull(compareBy<Request> { it.priority }.thenBy { it.revision })
            ?: return LEDColor.OFF

        return request.colorAt(now)
    }

    private fun Request.colorAt(now: Long): LEDColor {
        return when (mode) {
            Mode.Glow -> color
            is Mode.Blink -> {
                val phase = ((now - startedAtMs) / mode.periodMs).toInt()
                if (phase % 2 == 0) color else LEDColor.OFF
            }
        }
    }

    private fun Request.isFinished(now: Long): Boolean {
        return mode is Mode.Blink && now - startedAtMs >= mode.times * 2L * mode.periodMs
    }

    fun setColor(c: LEDColor) {
        if (displayedColor != c) {
            displayedColor = c
            led.position = c.pos
        }
    }
}
