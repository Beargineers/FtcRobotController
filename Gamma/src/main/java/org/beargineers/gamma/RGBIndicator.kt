package org.beargineers.gamma

import com.qualcomm.robotcore.hardware.Servo
import org.beargineers.platform.Hardware

enum class LEDColor(val pos: Double) {
    OFF(0.0),
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


class RGBIndicator(val bot: GammaRobot): Hardware(bot) {
    val led by hardware<Servo>()

    override fun init() {
        setColor(LEDColor.WHITE)
    }

    private var oldArtifactCount = 50
    override fun loop() {
        if (oldArtifactCount != bot.artifactsCount) {
            oldArtifactCount = bot.artifactsCount
            if (oldArtifactCount == 3) {
                setColor(LEDColor.GREEN)
            }
            else {
                setColor(LEDColor.WHITE)
            }
        }
    }

    fun setColor(c: LEDColor) {
        led.position = c.pos
    }
}