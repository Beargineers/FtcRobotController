package org.beargineers.platform

import org.beargineers.platform.decode.DecodeRobot
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import java.util.Locale
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

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

val BLUE_OPEN_GATE = tileLocation("A3", tileOffset = TileOffset.TOP_CENTER)
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
        return String.format(Locale.getDefault(), "(%.3f %.3f)%s", x, y, unit.toString())
    }
}

class Distance( val distance: Double, val distanceUnit: DistanceUnit = DISTANCE_UNIT){
    fun toDistanceUnit(unit: DistanceUnit): Distance {
        return Distance(unit.fromUnit(distanceUnit, distance), unit)
    }

    // TODO: fun add (take in the distance to add, and distance unit of the add)
}

class Angle(val angle: Double, val angleUnit: AngleUnit = ANGLE_UNIT){
    fun toAngleUnit(unit: AngleUnit): Angle{
        return Angle(unit.fromUnit(angleUnit, angle), unit)
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

fun Position.toAbsolute(): Position {
    val h = toAngleUnit(AngleUnit.RADIANS).heading
    val magnitude = hypot(x, y)
    val absoluteH = atan2(y,x)
    val absH = absoluteH + h
    val absX = cos(absH)*magnitude
    val absY = sin(absH)*magnitude
    return Position(absX, absY, absH, angleUnit = AngleUnit.RADIANS)
}

fun Position.toRelative(): Position {
    val h = toAngleUnit(AngleUnit.RADIANS).heading
    val magnitude = hypot(x, y)
    val absoluteH = atan2(y,x)
    val relH = absoluteH - h
    val relX = cos(relH)*magnitude
    val relY = sin(relH)*magnitude
    return Position(relX, relY, relH, angleUnit = AngleUnit.RADIANS)
}
fun Location.toAbsolute(heading: Angle): Location {
    val h = heading.toAngleUnit(AngleUnit.RADIANS).angle
    val magnitude = hypot(x, y)
    val absoluteH = atan2(y,x)
    val absH = absoluteH + h
    val absX = cos(absH)*magnitude
    val absY = sin(absH)*magnitude
    return Location(absX, absY, unit)
}

fun Location.toRelative(heading: Angle): Location {
    val h = heading.toAngleUnit(AngleUnit.RADIANS).angle
    val magnitude = hypot(x, y)
    val absoluteH = atan2(y,x)
    val relH = absoluteH - h
    val relX = cos(relH)*magnitude
    val relY = sin(relH)*magnitude
    return Location(relX, relY, unit)
}

fun DecodeRobot.goalDistanceCM(): Double {
    val goalCords =
        (if (opMode.alliance == Alliance.BLUE) BLUE_GOAL else RED_GOAL).toUnit(DistanceUnit.CM)

    val cp = currentPosition.toDistanceUnit(DistanceUnit.CM)
    val goalDistanceCM = hypot(cp.x - goalCords.x, cp.y - goalCords.y)
    return goalDistanceCM
}

fun DecodeRobot.shootingAngleCorrectionForMovement() : Double {
    return 0.0
}

fun DecodeRobot.headingToGoal(): Double {
    val goal = (if (opMode.alliance == Alliance.BLUE) BLUE_GOAL else RED_GOAL).toUnit(DistanceUnit.CM)
    val cp = currentPosition.toDistanceUnit(DistanceUnit.CM)
    val dx = goal.x - cp.x
    val dy = goal.y - cp.y
    return atan2(dy, dx) + shootingAngleCorrectionForMovement()
}

fun DecodeRobot.clearForShooting(): Boolean{
    fun pointInShootingZone(point: Location): Boolean{
        val p: Location = point.toUnit(distanceUnit = DistanceUnit.INCH)
        return if (p.x > 0){ // far shooting zones
            if (p.y > 0){ // red far shooting zone
                p.y < p.x - 48
            }else{ // Blue far shooting zone
                -p.y < p.x -48
            }
        }else{ // close shooting zone
            if (p.y > 0){// red
                p.y < -p.x
            }else{ // blue
                -p.y < -p.x
            }

        }
    }

    fun inShootingZone(): Boolean{
        val xFromCenter = 7.0
        val yFromCenter = 7.0
        val fr = Location(xFromCenter,yFromCenter, DistanceUnit.CM).toAbsolute(Angle(currentPosition.heading, currentPosition.angleUnit))
        val fl = Location(-xFromCenter,yFromCenter, DistanceUnit.CM).toAbsolute(Angle(currentPosition.heading, currentPosition.angleUnit))
        val br = Location(xFromCenter,-yFromCenter, DistanceUnit.CM).toAbsolute(Angle(currentPosition.heading, currentPosition.angleUnit))
        val bl = Location(-xFromCenter,-yFromCenter, DistanceUnit.CM).toAbsolute(Angle(currentPosition.heading, currentPosition.angleUnit))
        return pointInShootingZone(fr) || pointInShootingZone(fl) || pointInShootingZone(br) || pointInShootingZone(bl)
    }

    fun headingIsAtGoal(): Boolean{
        val sideDistanceDeviation = Distance(15.0, distanceUnit = DistanceUnit.CM)
        val distanceToGoal = Distance(goalDistanceCM(), distanceUnit = DistanceUnit.CM)
        val maxHeadingDeviation: Double = atan2(sideDistanceDeviation.toDistanceUnit(distanceToGoal.distanceUnit).distance, distanceToGoal.distance)
        return currentPosition.heading > headingToGoal() - maxHeadingDeviation && currentPosition.heading < headingToGoal() + maxHeadingDeviation
    }

    // TODO: add a check if the speed if the flywheel is good
    return inShootingZone() && headingIsAtGoal()
}
