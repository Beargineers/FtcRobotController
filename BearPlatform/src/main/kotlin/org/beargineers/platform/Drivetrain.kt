package org.beargineers.platform

import java.util.Locale

data class RelativePosition(val forward: Distance, val right: Distance, val turn: Angle) {
    operator fun plus(other: RelativePosition): RelativePosition {
        return RelativePosition(forward + other.forward, right + other.right, turn + other.turn)
    }

    override fun toString(): String {
        return String.format(Locale.getDefault(), "(%s %s) %s", forward, right, turn)
    }

    companion object {
        fun zero(): RelativePosition {
            return RelativePosition(0.cm, 0.cm, 0.degrees)
        }

        fun forward(d: Distance): RelativePosition {
            return RelativePosition(d, 0.cm, 0.degrees)
        }

        fun right(d: Distance): RelativePosition {
            return RelativePosition(0.cm, d, 0.degrees)
        }

        fun turnCCW(angle: Angle): RelativePosition {
            return RelativePosition(0.cm, 0.cm, angle)
        }
    }
}

interface Drivetrain {
    fun stop()
    fun drive(forwardPower: Double, rightPower: Double, turnPower: Double)
    fun driveByPowerAndAngle(theta: Double, power: Double, turn: Double)
}