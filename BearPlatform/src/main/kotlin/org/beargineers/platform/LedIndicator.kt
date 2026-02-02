package org.beargineers.platform

import com.qualcomm.robotcore.hardware.LED
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

data class LEDPattern(val phases: List<LEDPhase>)
data class LEDPhase(val pattern: String, val duration: Duration)

fun blink(pattern: String, period: Duration = 300.milliseconds): LEDPattern {
    return LEDPattern(buildList {
        add(LEDPhase(pattern, period))
        add(LEDPhase("000", period))
    })
}

fun counter(n: Int, color: Char): LEDPattern {
    if (n > 3) return blink("$color$color$color")
    return LEDPattern(buildList {
        add(LEDPhase(buildString {
            repeat(n) { append(color) }
            append("000")
        }.take(3), 100.seconds))
    })
}


class LedIndicator(robot: BaseRobot) : Hardware(robot) {
    private var basePattern = LEDPattern(buildList { add(LEDPhase("000", 100.seconds)) })
    private var tempPattern : LEDPattern? = null
    private var stopTempAtMillis = 0L
    private var startTime = 0L
    private var loopTime = 1L


    fun setBasePattern(pattern: LEDPattern) {
        if (basePattern != pattern) {
            basePattern = pattern
            startTime = System.currentTimeMillis()
            startPattern(pattern)
        }
    }

    fun setTempPattern(pattern: LEDPattern, duration: Duration) {
        if (tempPattern != pattern) {
            tempPattern = pattern
            stopTempAtMillis = System.currentTimeMillis() + duration.inWholeMilliseconds
            startPattern(pattern)
        }
    }

    private fun startPattern(pattern: LEDPattern) {
        startTime = System.currentTimeMillis()
        loopTime = pattern.phases.sumOf { it.duration.inWholeMilliseconds }
    }

    fun currentPattern(phases: List<LEDPhase>): String {
        val offset = (System.currentTimeMillis() - startTime) % loopTime
        phases.fold(0L) { acc, phase ->
            val t = acc + phase.duration.inWholeMilliseconds
            if (t > offset) {
                return phase.pattern
            }

            t
        }

        return "000"
    }

    val led1 = Led("led1")
    val led2 = Led("led2")
    val led3 = Led("led3")

    override fun init() {
        led1.color('0')
        led2.color('0')
        led3.color('0')
    }

    override fun loop() {
        val now = System.currentTimeMillis()

        if (tempPattern != null && now >= stopTempAtMillis) {
            tempPattern = null
            startPattern(basePattern)
        }

        val pattern = tempPattern ?: basePattern

        val colors = currentPattern(pattern.phases)
        led1.color(colors[0])
        led2.color(colors[1])
        led3.color(colors[2])
    }

    override fun stop() {
        led1.color('R')
        led2.color('R')
        led3.color('R')
    }

    inner class Led(name: String) {
        private val red = getHardware<LED>(name + "red")
        private val green = getHardware<LED>(name + "green")
        private var state: Char = ' '

        fun color(s: Char) {
            if (state != s) {
                state = s
                when (s) {
                    'R' -> {
                        red.on()
                        green.off()
                    }

                    'G' -> {
                        red.off()
                        green.on()
                    }

                    'Y' -> {
                        red.on()
                        green.on()
                    }

                    '0', 'O' -> {
                        red.off()
                        green.off()
                    }
                }
            }
        }
    }
}