package org.beargineers.platform.decode

import org.beargineers.platform.Alliance
import org.beargineers.platform.Angle
import org.beargineers.platform.Distance
import org.beargineers.platform.Location
import org.beargineers.platform.Position
import org.beargineers.platform.Robot
import org.beargineers.platform.RobotCentricLocation
import org.beargineers.platform.RobotCentricPosition
import org.beargineers.platform.RobotDimensions
import org.beargineers.platform.StateHolder
import org.beargineers.platform.Waypoint
import org.beargineers.platform.abs
import org.beargineers.platform.atan2
import org.beargineers.platform.between
import org.beargineers.platform.buildPath
import org.beargineers.platform.cm
import org.beargineers.platform.coerceInFieldBounds
import org.beargineers.platform.config
import org.beargineers.platform.degrees
import org.beargineers.platform.headingFromTo
import org.beargineers.platform.hypot
import org.beargineers.platform.inch
import org.beargineers.platform.isWithinFieldBounds
import org.beargineers.platform.max
import org.beargineers.platform.min
import org.beargineers.platform.tileLocation
import org.beargineers.platform.times
import org.beargineers.platform.toFieldCentric
import kotlin.math.sign
import kotlin.math.sqrt

interface DecodeRobot : Robot {
    suspend fun followPathAndShoot(waypoints: List<Waypoint>, applyMirroring: Boolean)

    suspend fun prepareForShutdown() {
        intakeMode = IntakeMode.OFF
        flywheelEnabled = false
    }

    fun isShooting(): Boolean

    fun adjustShooting(distance: Double, angle: Double)

    val shootingAngleCorrection: Angle

    val artifactsCount: Int

    val hasTurret: Boolean

    val hasVision: Boolean get() = false

    fun intakeTarget(filter: (Location) -> Boolean): Location? = null

    /**
     *  Angle at which an artifact would be fired if it happens now. Naturally, it is a heading of a turret
     *  if the robot has it or just a heading of a robot if it shoots forward.
     */
    val shooterAngle: Angle get() = currentPosition.heading

    fun shooterIsReady(): Boolean = flywheelEnabled

    fun resetTurret() {}
}

fun DecodeRobot.headingIsAtGoal(): Boolean {
    val sideDistanceDeviation = 15.cm
    val distanceToGoal = goalDistance()
    val maxHeadingDeviation = atan2(sideDistanceDeviation, distanceToGoal)
    val headingToGoal = headingToGoal()
    val shootingAngle = shooterAngle
    return abs((shootingAngle - headingToGoal).normalize()) <= maxHeadingDeviation
}

val IntakeState = StateHolder("Intake", IntakeMode.OFF)
var DecodeRobot.intakeMode by IntakeState
var DecodeRobot.flywheelEnabled by StateHolder("Flywheel", false)

object Locations {
    val OPEN_RAMP_SPEED by config(0.6)

    val GOAL by config(tileLocation("F6TR"))
    val PARK by config(tileLocation("B2BR").shift(-9.inch, -9.inch))
    val OPEN_RAMP_COLLECT by config(-6.cm, 129.cm, 90.degrees)
    val OPEN_RAMP_COLLECT_APPROACH by config(-6.cm, 80.cm, 90.degrees)
    val COLLECT_FROM_OPEN_RAMP by config(50.cm, 152.cm, 152.degrees)
    val COLLECT_FROM_OPEN_RAMP_APPROACH by config(50.cm, 132.cm, 90.degrees)

    val OPEN_RAMP by config(-6.cm, 129.cm, 90.degrees)
    val OPEN_RAMP_APPROACH by config(-6.cm, 80.cm, 90.degrees)

    val SPIKE_APPROACH_Y by config(61.0)
    val SPIKE_FINAL_Y by config(143.0) // 20cm less for Spike#3

    val SPIKE_SCOOPING_SPEED by config(1.0)
    val INITIAL_SHOT_SPEED by config(1.0)
}

fun Position.mirrorForAlliance(alliance: Alliance): Position {
    return if (alliance == Alliance.RED) this else Position(x, -y, -heading)
}

fun Location.mirrorForAlliance(alliance: Alliance): Location {
    return if (alliance == Alliance.RED) this else Location(x, -y)
}

fun DecodeRobot.goalDistance(): Distance {
    return goalDistanceFrom(currentPosition)
}

fun DecodeRobot.goalDistanceFrom(pos: Position): Distance {
    val goal = Locations.GOAL.mirrorForAlliance(alliance)
    return hypot(pos.x - goal.x, pos.y - goal.y)
}

fun DecodeRobot.headingToGoal(): Angle {
    return headingToGoalFrom(currentPosition.location())
}

fun DecodeRobot.headingToGoalFrom(position: Location): Angle {
    val goal = Locations.GOAL.mirrorForAlliance(alliance)
    return headingFromTo(position, goal) + shootingAngleCorrection
}

enum class ShootingZones {
    CLOSEST, FRONT, BACK
}

private object ShootingZoneOptimizationConfig {
    val zoneSafetyMargin by config(5.cm)
    val minGoalDistance by config(80.cm)
}

private fun robotSamplePoints(position: Position): List<Location> {
    val halfWidth = RobotDimensions.ROBOT_WIDTH / 2
    return listOf(
        RobotCentricLocation(RobotDimensions.ROBOT_FRONT_OFFSET, halfWidth),
        RobotCentricLocation(RobotDimensions.ROBOT_FRONT_OFFSET, -halfWidth),
        RobotCentricLocation(-RobotDimensions.ROBOT_BACK_OFFSET, halfWidth),
        RobotCentricLocation(-RobotDimensions.ROBOT_BACK_OFFSET, -halfWidth),
        RobotCentricLocation(RobotDimensions.ROBOT_FRONT_OFFSET, 0.cm),
        RobotCentricLocation(-RobotDimensions.ROBOT_BACK_OFFSET, 0.cm),
        RobotCentricLocation(0.cm, halfWidth),
        RobotCentricLocation(0.cm, -halfWidth),
        RobotCentricLocation.zero()
    ).map { it.toFieldCentric(position) }
}

private fun pointInCloseShootingZone(point: Location, margin: Distance = 0.cm): Boolean {
    return -point.x - abs(point.y) >= margin
}

private fun pointInFarShootingZone(point: Location, margin: Distance = 0.cm): Boolean {
    return point.x - 48.inch - abs(point.y) >= margin
}

private fun pointInShootingZone(point: Location, shootingZone: ShootingZones, margin: Distance = 0.cm): Boolean {
    return when (shootingZone) {
        ShootingZones.CLOSEST -> pointInCloseShootingZone(point, margin) || pointInFarShootingZone(point, margin)
        ShootingZones.FRONT -> pointInCloseShootingZone(point, margin)
        ShootingZones.BACK -> pointInFarShootingZone(point, margin)
    }
}

private fun DecodeRobot.retreatIntoAllianceHalf(position: Position): Position {
    val robotPoints = robotSamplePoints(position)
    val shiftY = if (alliance == Alliance.RED) {
        val minY = robotPoints.minOf { it.y } - ShootingZoneOptimizationConfig.zoneSafetyMargin
        if (minY < 0.cm) -minY else 0.cm
    } else {
        val maxY = robotPoints.maxOf { it.y } + ShootingZoneOptimizationConfig.zoneSafetyMargin
        if (maxY > 0.cm) -maxY else 0.cm
    }

    return position.shift(sign(position.x.distance) * abs(shiftY), shiftY).coerceInFieldBounds()
}

private fun DecodeRobot.travelHeadingTo(from: Position, to: Location): Angle {
    if (!hasTurret) {
        return headingToGoalFrom(to)
    }

    val headingTo = headingFromTo(from.location(), to)
    val tangential = listOf(headingTo, (headingTo + 180.degrees).normalize()).minBy { abs((from.heading - it).normalize()) }

    val diff = abs((from.heading - tangential).normalize())
    val distance = from.distanceTo(to)

    // Heuristics. If we have to turn more than 1 degree for each 2 cm of travel then let's just keep original heading and strafe
    return if (2 * distance.cm() > diff.degrees()) tangential else from.heading
}

private fun DecodeRobot.isPositionCloseToLever(pos: Position): Boolean {
    if (pos.x < 0.cm) return false

    val rampLever = tileLocation("F3TR").shift(0.cm, -20.cm).mirrorForAlliance(alliance)
    val clearance = pos.distanceTo(rampLever)
    return clearance < RobotDimensions.ROBOT_LENGTH
}

private fun DecodeRobot.requiredProtectionBackoff(
    targetLocation: Location,
    startPosition: Position,
    protectedZones: List<Location>
): Distance {
    var maxBackoff = 0.cm
    if (isPositionCloseToLever(startPosition)) {
        maxBackoff = 40.cm
    }

    val minX = min(startPosition.x, targetLocation.x)
    val maxX = max(startPosition.x, targetLocation.x)
    for (zone in protectedZones) {
        if (zone.x in minX..maxX) {
            maxBackoff = max(maxBackoff, abs(zone.y))
        }
    }

    return maxBackoff
}


fun DecodeRobot.planShootingApproach(
    shootingZone: ShootingZones,
    startPosition: Position,
    protectedZones: List<Location>,
    stayInAllianceHalf: Boolean
): List<Waypoint> {
    val firstCandidate = closestPointInShootingZone(shootingZone, startPosition.location())
    val backoff = requiredProtectionBackoff(firstCandidate, startPosition, protectedZones)
    val backedOffStart = startPosition.shift(0.cm, -backoff * alliance.sign)

    val secondCandidate = closestPointInShootingZone(shootingZone, backedOffStart.location())
    val targetHeading = travelHeadingTo(backedOffStart, secondCandidate)

    val target = findClosestPointOnLine(
        backedOffStart.location(),
        secondCandidate.withHeading(targetHeading).location()) {
        val pos = it.withHeading(targetHeading)
        inShootingZone(pos, shootingZone)
    }.withHeading(targetHeading).let {
        if (stayInAllianceHalf) retreatIntoAllianceHalf(it) else it
    }.let {
        moveAwayFromGoal(it)
    }

    val path = buildPath {
        if (backedOffStart != startPosition) {
            addRelaxedWaypoint(backedOffStart)
        }

        if (hasTurret) {
            addRelaxedWaypoint(target)
        }
        else {
            addStrictHeadingWaypoint(target)
        }
    }

    return path
}

fun DecodeRobot.inShootingZone(shootingZone: ShootingZones = ShootingZones.CLOSEST): Boolean {
    val cp = currentPosition
    return inShootingZone(cp, shootingZone)
}

fun inShootingZone(position: Position, shootingZone: ShootingZones): Boolean {
    return robotSamplePoints(position).any {
        pointInShootingZone(it, shootingZone, ShootingZoneOptimizationConfig.zoneSafetyMargin)
    }
}

fun DecodeRobot.clearForShooting(): String? {
    if (!headingIsAtGoal()) return "AIMING IS INCORRECT"

    if (!flywheelEnabled) return "FLYWHEEL IS OFF"
    if (!inShootingZone()) return "NOT IN SHOOTING ZONE"


    if (!shooterIsReady()) return "SHOOTER IS NOT READY"

    return null
}

fun DecodeRobot.moveAwayFromGoal(pos: Position): Position {
    val goal = Locations.GOAL.mirrorForAlliance(alliance)
    val clearance = goal.distanceTo(pos) - ShootingZoneOptimizationConfig.minGoalDistance
    if (clearance >= 0.cm) return pos

    val allowedZones = listOf(ShootingZones.FRONT, ShootingZones.BACK).filter { inShootingZone(pos, it) }
        .ifEmpty { listOf(ShootingZones.FRONT, ShootingZones.BACK) }

    fun isValid(candidate: Position): Boolean {
        return goal.distanceTo(candidate) >= ShootingZoneOptimizationConfig.minGoalDistance &&
            allowedZones.any { inShootingZone(candidate, it) } &&
            robotSamplePoints(candidate).all { it.isWithinFieldBounds() }
    }

    fun attempt(
        dxPerStep: Double,
        dyPerStep: Double,
        move: (Distance) -> Position
    ): Position? {
        val relX = (pos.x - goal.x).cm()
        val relY = (pos.y - goal.y).cm()
        val minDistance = ShootingZoneOptimizationConfig.minGoalDistance.cm()
        val a = dxPerStep * dxPerStep + dyPerStep * dyPerStep
        val b = 2 * (relX * dxPerStep + relY * dyPerStep)
        val c = relX * relX + relY * relY - minDistance * minDistance
        val discriminant = b * b - 4 * a * c
        if (discriminant < 0.0) return null

        val step = ((-b + sqrt(discriminant)) / (2 * a)).cm
        return move(step).takeIf(::isValid)
    }

    val oneCentimeterForward = pos + RobotCentricPosition(1.cm, 0.cm, 0.degrees)
    val forwardDx = (oneCentimeterForward.x - pos.x).cm()
    val forwardDy = (oneCentimeterForward.y - pos.y).cm()
    attempt(forwardDx, forwardDy) { pos + RobotCentricPosition(it, 0.cm, 0.degrees) }?.let { return it }
    attempt(-forwardDx, -forwardDy) { pos + RobotCentricPosition(-it, 0.cm, 0.degrees) }?.let { return it }
    attempt(1.0, 0.0) { pos.shift(it, 0.cm) }?.let { return it }

    val ySign = -sign(goal.y.distance)
    attempt(1.0, ySign) { pos.shift(it, ySign * it) }?.let { return it }

    return pos
}

fun DecodeRobot.closestPointInFrontZone(from: Location): Location {
    if (inShootingZone(ShootingZones.FRONT)) return from

    val (x, y) = from
    val close = if (y <= 0.cm) { // blue side
        if (y < -x) {
            Location((x + y) * 0.5, (x + y) * 0.5)
        } else {
            Location(0.cm, 0.cm)
        }
    } else { // RED side
        if (y > x) {
            Location(-(y - x) * 0.5, (y - x) * 0.5)
        } else {
            Location(0.cm, 0.cm)
        }
    }

    return close
}

fun findClosestPointOnLine(from: Location, to: Location, f: (Location) -> Boolean ): Location {
    var left = from
    var right = to

    repeat(10) {
        val mid = left.between(right)
        if (f(mid)) {
            right = mid
        } else {
            left = mid
        }
    }

    return right
}

fun DecodeRobot.closestPointInBackZone(from: Location): Location {
    if (inShootingZone(ShootingZones.BACK)) return from
    return tileLocation("D1CL")
}

fun DecodeRobot.closestPointInShootingZone(shootingZone: ShootingZones, from: Location = currentPosition.location()): Location {
    val close = closestPointInFrontZone(from)
    val far = closestPointInBackZone(from)

    return when (shootingZone) {
        ShootingZones.CLOSEST -> if (from.distanceTo(close) < from.distanceTo(far)) close else far
        ShootingZones.FRONT -> close
        ShootingZones.BACK -> far
    }
}


fun spikeStart(n: Int): Position {
    val x = ((2 - n)*24).inch + 12.inch
    val y = Locations.SPIKE_APPROACH_Y

    return Position(x, y.cm, 90.degrees)
}

fun spikeEnd(n: Int): Position {
    val x = ((2 - n)*24).inch + 12.inch
    val y = if (n == 3) (Locations.SPIKE_FINAL_Y - 20) else Locations.SPIKE_FINAL_Y

    return Position(x, y.cm, 90.degrees)
}
