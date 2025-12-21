package org.beargineers.platform

import org.beargineers.platform.decode.DecodeRobot
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import java.util.Locale
import kotlin.math.PI
import kotlin.math.abs
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

val FIELD_CENTER = Location(0.cm, 0.cm).withHeading(0.degrees)

val RED_GOAL = Location(-58.3727.inch, 55.6425.inch)

val BLUE_GOAL = Location(-58.3727.inch, -55.6425.inch)

val RED_PARK = tileLocation("B2", tileOffset = TileOffset.BOTTOM_RIGHT).shift(-9.inch, -9.inch)

val BLUE_PARK = tileLocation("E2", tileOffset = TileOffset.BOTTOM_LEFT).shift(-9.inch, 9.inch)

val RED_OPEN_GATE = tileLocation("F3", tileOffset = TileOffset.TOP_CENTER)

val BLUE_OPEN_GATE = tileLocation("A3", tileOffset = TileOffset.TOP_CENTER)
class Location(val x: Distance, val y: Distance) {
    fun shift(x: Distance, y: Distance): Location {
        return Location(this.x + x, this.y + y)
    }

    fun withHeading(heading: Angle): Position {
        return Position(x, y, heading)
    }

    override fun toString(): String {
        return String.format(Locale.getDefault(), "(%s %s)", x, y)
    }

    fun distanceTo(other: Position) : Distance {
        val Other: Location = Location(other.x, other.y)
        return Distance(abs(hypot((abs(x.cm()) - abs(Other.x.cm())), (abs(y.cm()) - abs(Other.y.cm())))),
            distanceUnit = DistanceUnit.CM)
    }

    fun distanceTo(other: Location) : Distance {
        val Other: Location = Location(other.x, other.y)
        return Distance(abs(hypot((abs(x.cm()) - abs(Other.x.cm())), (abs(y.cm()) - abs(Other.y.cm())))),
            distanceUnit = DistanceUnit.CM)
    }
}

class Distance( val distance: Double, val distanceUnit: DistanceUnit) : Comparable<Distance> {
    override fun toString(): String {
        return String.format(Locale.US, "%.3f%s", distance, distanceUnit)
    }

    operator fun plus(other: Distance): Distance {
        return Distance(distance + distanceUnit.fromUnit(other.distanceUnit, other.distance), distanceUnit)
    }

    operator fun minus(other: Distance): Distance {
        return Distance(distance - distanceUnit.fromUnit(other.distanceUnit, other.distance), distanceUnit)
    }

    operator fun times(other: Double): Distance {
        return Distance(distance * other, distanceUnit)
    }

    operator fun div(other: Double): Distance {
        return Distance(distance / other, distanceUnit)
    }

    operator fun div(other: Distance): Double {
        return distance / distanceUnit.fromUnit(other.distanceUnit, other.distance)
    }

    operator fun unaryMinus(): Distance {
        return Distance(-distance, distanceUnit)
    }

    override fun compareTo(other: Distance): Int {
        return distance.compareTo(distanceUnit.fromUnit(other.distanceUnit, other.distance))
    }

    fun cm() : Double = DistanceUnit.CM.fromUnit(distanceUnit, distance)
    fun inch(): Double = DistanceUnit.INCH.fromUnit(distanceUnit, distance)
}

operator fun Double.times(other: Distance): Distance = other * this
operator fun Double.div(other: Distance): Distance = other / this

val Double.cm: Distance get() =  Distance(this, DistanceUnit.CM)
val Double.inch: Distance get() = Distance(this, DistanceUnit.INCH)
val Int.cm: Distance get() = Distance(this.toDouble(), DistanceUnit.CM)
val Int.inch: Distance get() = Distance(this.toDouble(), DistanceUnit.INCH)

fun atan2(y: Distance, x: Distance): Angle {
    return Angle(atan2(y.cm(), x.cm()), AngleUnit.RADIANS)
}

fun hypot(x: Distance, y: Distance): Distance {
    return Distance(hypot(x.cm(), y.cm()), DistanceUnit.CM)
}

fun abs(x: Distance): Distance {
    return Distance(abs(x.distance), x.distanceUnit)
}

class Angle(val angle: Double, val angleUnit: AngleUnit) : Comparable<Angle> {
    override fun toString(): String {
        return String.format(Locale.US, "%.3f%s", angle, angleUnit)
    }

    operator fun plus(other: Angle): Angle {
        return Angle(angle + angleUnit.fromUnit(other.angleUnit, other.angle), angleUnit)
    }

    operator fun minus(other: Angle): Angle {
        return Angle(angle - angleUnit.fromUnit(other.angleUnit, other.angle), angleUnit)
    }

    operator fun times(other: Double): Angle {
        return Angle(angle * other, angleUnit)
    }

    operator fun div(other: Double): Angle {
        return Angle(angle / other, angleUnit)
    }

    operator fun div(other: Angle): Double {
        return angle / angleUnit.fromUnit(other.angleUnit, other.angle)
    }

    operator fun unaryMinus(): Angle {
        return Angle(-angle, angleUnit)
    }

    override fun compareTo(other: Angle): Int {
        return angle.compareTo(angleUnit.fromUnit(other.angleUnit, other.angle))
    }

    fun normalize(): Angle {
        var answer = this
        val halfCircle = PI.radians
        val fullCircle = halfCircle * 2.0
        // Normalize heading error to [-180, 180] or [-π, π]
        while (answer > halfCircle) answer -= fullCircle
        while (answer < -halfCircle) answer += fullCircle

        return answer
    }

    fun radians(): Double = AngleUnit.RADIANS.fromUnit(angleUnit, angle)
    fun degrees(): Double = AngleUnit.DEGREES.fromUnit(angleUnit, angle)
}

fun sin(theta: Angle): Double {
    return sin(theta.radians())
}

fun cos(theta: Angle): Double {
    return cos(theta.radians())
}

fun abs(theta: Angle): Angle {
    return Angle(abs(theta.angle), theta.angleUnit)
}

val Double.degrees: Angle get() = Angle(this, AngleUnit.DEGREES)
val Double.radians: Angle get() = Angle(this, AngleUnit.RADIANS)
val Int.degrees: Angle get() = Angle(this.toDouble(), AngleUnit.DEGREES)

operator fun Double.times(other: Angle): Angle = other * this
operator fun Double.div(other: Angle): Angle = other / this

class Position(val x: Distance, val y: Distance, val heading: Angle) {
    fun rotate(angle: Angle): Position {
        return Position(x, y, heading + angle)
    }

    fun shift(x: Distance, y: Distance): Position {
        return Position(this.x + x, this.y + y, heading)
    }

    fun location() : Location = Location(x, y)

    operator fun plus(other: Position): Position {
        return Position(x + other.x, y + other.y, heading + other.heading).normalizeHeading()
    }

    operator fun minus(other: Position): Position {
        return Position(x - other.x, y - other.y, heading - other.heading).normalizeHeading()
    }

    fun normalizeHeading() : Position {
        var h = heading
        while (h < -PI.radians) h += (2* PI).radians
        while (h > PI.radians) h -= (2* PI).radians
        return Position(x, y, h)
    }

    override fun toString(): String {
        return String.format(Locale.getDefault(), "(%s %s) %s", x, y, heading)
    }

    companion object {
        fun zero() : Position = Position(0.cm, 0.cm, 0.degrees)
    }

    fun distanceTo(other: Position) : Distance {
        val Other: Location = Location(other.x, other.y)
        return Distance(abs(hypot((abs(x.cm()) - abs(Other.x.cm())), (abs(y.cm()) - abs(Other.y.cm())))),
            distanceUnit = DistanceUnit.CM)
    }

    fun distanceTo(other: Location) : Distance {
        val Other: Location = Location(other.x, other.y)
        return Distance(abs(hypot((abs(x.cm()) - abs(Other.x.cm())), (abs(y.cm()) - abs(Other.y.cm())))),
            distanceUnit = DistanceUnit.CM)
    }
}

fun Position.toAbsolute(): Position {
    val magnitude = hypot(x, y)
    val absoluteH = atan2(y,x)
    val absH = absoluteH + heading
    val absX = magnitude * cos(absH)
    val absY = magnitude * sin(absH)
    return Position(absX, absY, absH)
}

fun Position.toRelative(): Position {
    val magnitude = hypot(x, y)
    val absoluteH = atan2(y,x)
    val relH = absoluteH - heading
    val relX = magnitude * cos(relH)
    val relY = magnitude * sin(relH)
    return Position(relX, relY, relH)
}
fun Location.toAbsolute(heading: Angle): Location {
    val magnitude = hypot(x, y)
    val absoluteH = atan2(y,x)
    val absH = absoluteH + heading
    val absX = cos(absH)*magnitude
    val absY = sin(absH)*magnitude
    return Location(absX, absY)
}

fun Location.toRelative(heading: Angle): Location {
    val magnitude = hypot(x, y)
    val absoluteH = atan2(y,x)
    val relH = absoluteH - heading
    val relX = cos(relH)*magnitude
    val relY = sin(relH)*magnitude
    return Location(relX, relY)
}

fun DecodeRobot.goalDistance(): Distance {
    val goalCords = if (opMode.alliance == Alliance.BLUE) BLUE_GOAL else RED_GOAL
    val cp = currentPosition
    return hypot(cp.x - goalCords.x, cp.y - goalCords.y)
}

fun DecodeRobot.shootingAngleCorrectionForMovement() : Angle {
    return 0.degrees
}

fun DecodeRobot.headingToGoal(): Angle {
    val goal = (if (opMode.alliance == Alliance.BLUE) BLUE_GOAL else RED_GOAL)
    val cp = currentPosition
    val dx = goal.x - cp.x
    val dy = goal.y - cp.y
    return atan2(dy, dx) + shootingAngleCorrectionForMovement()
}

fun DecodeRobot.clearForShooting(): Boolean{
    fun pointInShootingZone(p: Location): Boolean{
        return if (p.x > 0.inch){ // far shooting zones
            if (p.y > 0.inch){ // red far shooting zone
                p.y < p.x - 48.inch
            }else{ // Blue far shooting zone
                -p.y < p.x -48.inch
            }
        }else{ // close shooting zone
            if (p.y > 0.inch){// red
                p.y < -p.x
            }else{ // blue
                -p.y < -p.x
            }
        }
    }

    fun inShootingZone(): Boolean{
        val xFromCenter = 7.cm
        val yFromCenter = 7.cm
        val fr = Location(xFromCenter,yFromCenter).toAbsolute(currentPosition.heading)
        val fl = Location(-xFromCenter,yFromCenter).toAbsolute(currentPosition.heading)
        val br = Location(xFromCenter,-yFromCenter).toAbsolute(currentPosition.heading)
        val bl = Location(-xFromCenter,-yFromCenter).toAbsolute(currentPosition.heading)
        return pointInShootingZone(fr) || pointInShootingZone(fl) || pointInShootingZone(br) || pointInShootingZone(bl)
    }

    fun headingIsAtGoal(): Boolean{
        val sideDistanceDeviation = 15.cm
        val distanceToGoal = goalDistance()
        val maxHeadingDeviation = atan2(sideDistanceDeviation, distanceToGoal)
        return currentPosition.heading > headingToGoal() - maxHeadingDeviation && currentPosition.heading < headingToGoal() + maxHeadingDeviation
    }

    fun correctSpeedOfFlywheel(): Boolean{
        return true
    }
    // TODO: add a check if the speed if the flywheel is good
    return inShootingZone() && headingIsAtGoal() && correctSpeedOfFlywheel()
}
