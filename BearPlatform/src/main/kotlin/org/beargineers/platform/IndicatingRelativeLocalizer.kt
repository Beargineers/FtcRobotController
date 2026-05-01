package org.beargineers.platform

import kotlin.time.Duration.Companion.seconds

class IndicatingRelativeLocalizer(val localizer: RelativeLocalizer, val led: LedIndicator) : RelativeLocalizer {
    override fun getPosition(oldPosition: Position): Position {
        return localizer.getPosition(oldPosition)
    }

    override fun updatePositionEstimate(position: Position) {
        led.setTempPattern(blink("RGR"), 2.seconds)
        localizer.updatePositionEstimate(position)
    }

    override fun getVelocity(): Position {
        return localizer.getVelocity()
    }
}