package org.beargineers.platform

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import java.util.Locale
import kotlin.math.PI

/*
Make sure to read https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html#square-field-inverted-alliance-area
It says that
- X axis goes from goals towards the audience, i.e. x getting higher when row index getting lower
- Y axis goes from blue goal side towards red goal side, i.e. y getting higher with column index getting higher
- Center of the field has [0,0] coordinates
- Heading is 0  when facing upwards, and increases counterclockwise [TODO:check by an experiment]
 */

val DISTANCE_UNIT = DistanceUnit.CM
val ANGLE_UNIT = AngleUnit.DEGREES

val FIELD_CENTER = Location(0.0, 0.0).withHeading(0.0, AngleUnit.DEGREES)

val RED_GOAL = Location(-58.3727, 55.6425, DistanceUnit.INCH)

val BLUE_GOAL = Location(-58.3727, -55.6425, DistanceUnit.INCH)

val RED_PARK = tileLocation("B2", tileOffset = TileOffset.BOTTOM_RIGHT).toUnit(DistanceUnit.INCH).shift(-9.0,-9.0)

val BLUE_PARK = tileLocation("E2", tileOffset = TileOffset.BOTTOM_LEFT).toUnit(DistanceUnit.INCH).shift(-9.0,9.0)

val RED_OPEN_GATE = tileLocation("F3", tileOffset = TileOffset.TOP_CENTER)

val BLUE_OPEN_GATE = tileLocation("A3", tileOffset = TileOffset.TOP_CENTER  )
class Location(
    val x: Double,
    val y: Double,
    val unit: DistanceUnit = DISTANCE_UNIT) {

    fun toUnit(distanceUnit: DistanceUnit): Location {
        return Location(
            distanceUnit.fromUnit(unit, x),
            distanceUnit.fromUnit(unit, y),
            distanceUnit)
    }

    fun shift(x: Double, y: Double): Location {
        return Location(this.x + x, this.y + y, unit)
    }

    fun withHeading(heading: Double, angleUnit: AngleUnit = ANGLE_UNIT): Position {
        return Position(x, y, heading, unit, angleUnit)
    }

    override fun toString(): String {
        return String.Companion.format(Locale.getDefault(), "(%.3f %.3f)%s", x, y, unit.toString())
    }
}

class Position(
    val x: Double,
    val y: Double,
    val heading: Double,
    val distanceUnit: DistanceUnit = DISTANCE_UNIT,
    val angleUnit: AngleUnit = ANGLE_UNIT
) {
    fun toDistanceUnit(unit: DistanceUnit): Position {
        return Position(
            unit.fromUnit(distanceUnit, x),
            unit.fromUnit(distanceUnit, y),
            heading,
            unit,
            angleUnit)
    }

    fun toAngleUnit(unit: AngleUnit): Position {
        return Position(
            x,
            y,
            unit.fromUnit(angleUnit, heading),
            distanceUnit,
            unit)
    }

    fun rotate(angle: Double): Position {
        return Position(x, y, heading + angle, distanceUnit, angleUnit)
    }

    fun shift(x: Double, y: Double): Position {
        return Position(this.x + x, this.y + y, heading, distanceUnit, angleUnit)
    }

    fun position() : Location = Location(x, y, distanceUnit)

    operator fun plus(other: Position): Position {
        val other = other.toDistanceUnit(distanceUnit).toAngleUnit(angleUnit)
        return Position(x + other.x, y + other.y, heading + other.heading, distanceUnit, angleUnit).normalizeHeading()
    }

    operator fun minus(other: Position): Position {
        val other = other.toDistanceUnit(distanceUnit).toAngleUnit(angleUnit)
        return Position(x - other.x, y - other.y, heading - other.heading, distanceUnit, angleUnit).normalizeHeading()
    }

    fun normalizeHeading() : Position {
        when (angleUnit) {
            AngleUnit.DEGREES -> {
                var h = heading
                while (h < -180) h += 360
                while (h > 180) h -= 360

                return Position(x, y, h, distanceUnit, angleUnit)
            }

            AngleUnit.RADIANS -> {
                var h = heading
                while (h < -PI) h += 2* PI
                while (h > PI) h -= 2* PI
                return Position(x, y, h, distanceUnit, angleUnit)
            }
        }
    }

    override fun toString(): String {
        return String.format(Locale.getDefault(), "(%.3f %.3f)%s  %.3f%s", x, y, distanceUnit, heading, angleUnit)
    }

    companion object {
        fun zero() : Position = Position(0.0, 0.0, 0.0)
    }
}