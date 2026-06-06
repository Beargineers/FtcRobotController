package org.beargineers.platform

import kotlin.time.Duration.Companion.milliseconds

object AprilTagSignal : RGBSignal {
    override val priority: Int = 40
}

class RGBIndicatingRelativeLocalizer(val localizer: RelativeLocalizer, val led: RGBIndicator) : RelativeLocalizer {
    override fun getPosition(oldPosition: Position): Position {
        return localizer.getPosition(oldPosition)
    }

    override fun updatePositionEstimate(position: Position) {
        led.blink(AprilTagSignal, LEDColor.ORANGE, 3, 200.milliseconds)
        localizer.updatePositionEstimate(position)
    }

    override fun getVelocity(): Position {
        return localizer.getVelocity()
    }
}