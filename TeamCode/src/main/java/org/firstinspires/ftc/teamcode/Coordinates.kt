package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import java.util.Locale

val DISTANCE_UNIT = DistanceUnit.CM
val ANGLE_UNIT = AngleUnit.DEGREES

/*
Make sure to read https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html#square-field-inverted-alliance-area
It says that
- X axis goes from goals towards the audience, i.e. x getting higher when row index getting lower
- Y axis goes from blue goal side towards red goal side, i.e. y getting higher with column index getting higher
- Center of the field has [0,0] coordinates
- Heading is 0  when facing upwards, and increases counterclockwise [TODO:check by an experiment]
 */

class Position2D(
    val x: Double,
    val y: Double,
    val unit: DistanceUnit = DISTANCE_UNIT) {

    fun toUnit(distanceUnit: DistanceUnit): Position2D {
        return Position2D(
            distanceUnit.fromUnit(unit, x),
            distanceUnit.fromUnit(unit, y),
            distanceUnit)
    }

    fun shift(x: Double, y: Double): Position2D {
        return Position2D(this.x + x, this.y + y, unit)
    }

    fun withHeading(heading: Double, angleUnit: AngleUnit = ANGLE_UNIT): Pose2D {
        return Pose2D(x, y, heading, unit, angleUnit)
    }

    override fun toString(): String {
        return String.format(Locale.getDefault(), "(%.3f %.3f)%s", x, y, unit.toString())
    }
}

class Pose2D(
    val x: Double,
    val y: Double,
    val heading: Double,
    val distanceUnit: DistanceUnit = DISTANCE_UNIT,
    val angleUnit: AngleUnit = ANGLE_UNIT
) {
    fun toDistanceUnit(unit: DistanceUnit): Pose2D {
        return Pose2D(
            unit.fromUnit(distanceUnit, x),
            unit.fromUnit(distanceUnit, y),
            heading,
            unit,
            angleUnit)
    }

    fun toAngleUnit(unit: AngleUnit): Pose2D {
        return Pose2D(
            x,
            y,
            unit.fromUnit(angleUnit, heading),
            distanceUnit,
            unit)
    }

    fun rotate(angle: Double): Pose2D {
        return Pose2D(x, y, heading + angle, distanceUnit, angleUnit)
    }

    fun shift(x: Double, y: Double): Pose2D {
        return Pose2D(this.x + x, this.y + y, heading, distanceUnit, angleUnit)
    }

    fun position() : Position2D = Position2D(x, y, distanceUnit)

    override fun toString(): String {
        return String.format(Locale.getDefault(), "(%.3f %.3f)%s  %.3f%s", x, y, distanceUnit, heading, angleUnit)
    }
}
