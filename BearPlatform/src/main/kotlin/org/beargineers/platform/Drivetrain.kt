package org.beargineers.platform

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import java.util.Locale

data class RobotMovement(val forward: Double, val right: Double, val turn: Double, val distanceUnit: DistanceUnit, val angleUnit: AngleUnit) {
    fun toUnits(distanceUnit: DistanceUnit, angleUnit: AngleUnit): RobotMovement {
        return RobotMovement(
            distanceUnit.fromUnit(this.distanceUnit, forward),
            distanceUnit.fromUnit(this.distanceUnit, right),
            angleUnit.fromUnit(this.angleUnit, turn),
            distanceUnit,
            angleUnit)
    }

    override fun toString(): String {
        return String.format(Locale.getDefault(), "(%.3f %.3f)%s  %.3f%s", forward, right, distanceUnit, turn, angleUnit)
    }

    companion object {
        fun zero(): RobotMovement {
            return RobotMovement(0.0, 0.0, 0.0, DistanceUnit.CM, AngleUnit.RADIANS)
        }

        fun forwardInch(inches: Double): RobotMovement {
            return RobotMovement(inches, 0.0, 0.0, DistanceUnit.INCH, AngleUnit.RADIANS)
        }

        fun rightInch(inches: Double): RobotMovement {
            return RobotMovement(0.0, inches, 0.0, DistanceUnit.INCH, AngleUnit.RADIANS)
        }

        fun turnCCW(degrees: Double): RobotMovement {
            return RobotMovement(0.0, 0.0, degrees, DistanceUnit.INCH, AngleUnit.DEGREES)
        }
    }
}

interface Drivetrain {
    fun stop()
    fun drive(forwardPower: Double, rightPower: Double, turnPower: Double, slow: Boolean = false)

    fun robotMovedBy(): RobotMovement
}