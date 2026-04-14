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
import org.beargineers.platform.RobotLocations
import org.beargineers.platform.StateHolder
import org.beargineers.platform.abs
import org.beargineers.platform.atan2
import org.beargineers.platform.between
import org.beargineers.platform.cm
import org.beargineers.platform.config
import org.beargineers.platform.degrees
import org.beargineers.platform.headingFromTo
import org.beargineers.platform.hypot
import org.beargineers.platform.inch
import org.beargineers.platform.isWithinFieldBounds
import org.beargineers.platform.tileLocation
import org.beargineers.platform.toFieldCentric
import org.beargineers.platform.toRobotCentric
import kotlin.math.max

interface DecodeRobot : Robot {
    suspend fun shoot(holdPosition: Boolean)
    suspend fun prepareForShooting()

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

    fun resetTurret() {}
}

var DecodeRobot.intakeMode by StateHolder("Intake", IntakeMode.OFF)
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
    val goal = Locations.GOAL.mirrorForAlliance(alliance)
    val cp = currentPosition
    return hypot(cp.x - goal.x, cp.y - goal.y)
}

fun DecodeRobot.headingToGoal(): Angle {
    return headingToGoalFrom(currentPosition.location())
}

fun DecodeRobot.headingToGoalFrom(position: Location): Angle {
    val goal = Locations.GOAL.mirrorForAlliance(alliance)
    val dx = goal.x - position.x
    val dy = goal.y - position.y
    return atan2(dy, dx) + shootingAngleCorrection
}

enum class ShootingZones {
    CLOSEST, FRONT, BACK
}

private object ShootingZoneOptimizationConfig {
    val zoneSafetyMargin by config(5.cm)
    val minGoalDistance by config(80.cm)
    val coarseSearchStep by config(5.cm)
    val fineSearchStep by config(1.cm)
    val fineSearchRadius by config(8.cm)
    val maxForwardSpeed by config(120.cm)
    val maxStrafeSpeed by config(60.cm)
    val maxTurnSpeed by config(180.degrees)
}

internal data class ShootingApproachPlan(
    val target: Position,
    val approachWaypoints: List<Position>,
    val estimatedTime: Double
)

private data class SearchBounds(
    val minX: Distance,
    val maxX: Distance,
    val minY: Distance,
    val maxY: Distance
)

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

private fun DecodeRobot.isOnOwnAllianceHalf(point: Location): Boolean {
    return if (alliance == Alliance.RED) {
        point.y >= 0.cm
    } else {
        point.y <= 0.cm
    }
}

private fun DecodeRobot.respectsAllianceHalf(position: Position): Boolean {
    return robotSamplePoints(position).all { isOnOwnAllianceHalf(it) }
}

private fun DecodeRobot.retreatIntoAllianceHalf(position: Position): Position {
    val robotPoints = robotSamplePoints(position)
    val shiftY = if (alliance == Alliance.RED) {
        val minY = robotPoints.minOf { it.y.cm() }
        if (minY < 0.0) (-minY).cm else 0.cm
    } else {
        val maxY = robotPoints.maxOf { it.y.cm() }
        if (maxY > 0.0) (-maxY).cm else 0.cm
    }
    return position.shift(0.cm, shiftY)
}

private fun DecodeRobot.isSafeShootingPose(
    position: Position,
    shootingZone: ShootingZones,
    stayInAllianceHalf: Boolean = false
): Boolean {
    val goal = Locations.GOAL.mirrorForAlliance(alliance)
    if (position.distanceTo(goal) < ShootingZoneOptimizationConfig.minGoalDistance) {
        return false
    }

    val robotPoints = robotSamplePoints(position)
    if (!robotPoints.all { it.isWithinFieldBounds() }) {
        return false
    }
    if (stayInAllianceHalf && !robotPoints.all { isOnOwnAllianceHalf(it) }) {
        return false
    }

    return robotPoints.any { pointInShootingZone(it, shootingZone, ShootingZoneOptimizationConfig.zoneSafetyMargin) }
}

private fun DecodeRobot.travelHeadingTo(from: Position, to: Location): Angle {
    if (!hasTurret) {
        return headingToGoalFrom(to)
    }

    val headingTo = headingFromTo(from.location(), to)
    return listOf(headingTo, (headingTo + 180.degrees).normalize())
        .minBy { abs((from.heading - it).normalize()) }
}

private fun DecodeRobot.requiredProtectionBackoff(
    targetLocation: Location,
    startPosition: Position,
    protectedZones: List<Location>
): Distance {
    var maxBackoff = 0.cm
    if (startPosition.distanceTo(Locations.OPEN_RAMP_COLLECT) < 10.cm ||
        startPosition.distanceTo(Locations.OPEN_RAMP) < 10.cm
    ) {
        maxBackoff = 15.cm
    }

    val minX = org.beargineers.platform.min(startPosition.x, targetLocation.x)
    val maxX = org.beargineers.platform.max(startPosition.x, targetLocation.x)
    for (zone in protectedZones) {
        if (zone.x in minX..maxX) {
            maxBackoff = org.beargineers.platform.max(maxBackoff, abs(zone.y) - abs(startPosition.y))
        }
    }

    return org.beargineers.platform.max(maxBackoff, 0.cm)
}

private fun estimateSegmentTime(from: Position, to: Position): Double {
    val relative = to.location().toRobotCentric(from)
    val translationTime = max(
        abs(relative.forward) / ShootingZoneOptimizationConfig.maxForwardSpeed,
        abs(relative.right) / ShootingZoneOptimizationConfig.maxStrafeSpeed
    )
    val rotationTime = abs((to.heading - from.heading).normalize()) / ShootingZoneOptimizationConfig.maxTurnSpeed
    return max(translationTime, rotationTime)
}

private fun DecodeRobot.evaluateShootingCandidate(
    shootingZone: ShootingZones,
    startPosition: Position,
    candidateLocation: Location,
    protectedZones: List<Location>,
    stayInAllianceHalf: Boolean
): ShootingApproachPlan? {
    val approachWaypoints = mutableListOf<Position>()
    fun addApproachWaypoint(position: Position) {
        if (approachWaypoints.lastOrNull() != position && startPosition != position) {
            approachWaypoints += position
        }
    }

    var planningStart = startPosition
    if (stayInAllianceHalf && !respectsAllianceHalf(planningStart)) {
        planningStart = retreatIntoAllianceHalf(planningStart)
        if (!respectsAllianceHalf(planningStart)) {
            return null
        }
        addApproachWaypoint(planningStart)
    }

    val backoff = requiredProtectionBackoff(candidateLocation, planningStart, protectedZones)
    var backedOffStart = planningStart + RobotCentricPosition(-backoff, 0.cm, 0.degrees)
    if (stayInAllianceHalf && !respectsAllianceHalf(backedOffStart)) {
        backedOffStart = retreatIntoAllianceHalf(backedOffStart)
        if (!respectsAllianceHalf(backedOffStart)) {
            return null
        }
    }
    addApproachWaypoint(backedOffStart)

    val target = candidateLocation.withHeading(travelHeadingTo(backedOffStart, candidateLocation))
    if (!isSafeShootingPose(target, shootingZone, stayInAllianceHalf)) {
        return null
    }

    val path = buildList {
        add(startPosition)
        addAll(approachWaypoints)
        add(target)
    }

    return ShootingApproachPlan(
        target = target,
        approachWaypoints = approachWaypoints,
        estimatedTime = path.zipWithNext().sumOf { (from, to) -> estimateSegmentTime(from, to) }
    )
}

private fun generateSearchGrid(bounds: SearchBounds, step: Distance): Sequence<Location> = sequence {
    var x = bounds.minX.cm()
    while (x <= bounds.maxX.cm() + 1e-9) {
        var y = bounds.minY.cm()
        while (y <= bounds.maxY.cm() + 1e-9) {
            yield(Location(x.cm, y.cm))
            y += step.cm()
        }
        x += step.cm()
    }
}

private fun searchBoundsFor(shootingZone: ShootingZones): SearchBounds {
    val fieldExtent = (24 * 3).inch
    val robotReach = org.beargineers.platform.max(
        org.beargineers.platform.max(RobotDimensions.ROBOT_FRONT_OFFSET, RobotDimensions.ROBOT_BACK_OFFSET),
        RobotDimensions.ROBOT_WIDTH / 2
    ) + ShootingZoneOptimizationConfig.zoneSafetyMargin

    return when (shootingZone) {
        ShootingZones.FRONT -> SearchBounds(
            minX = -fieldExtent,
            maxX = robotReach,
            minY = -fieldExtent,
            maxY = fieldExtent
        )
        ShootingZones.BACK -> SearchBounds(
            minX = 48.inch - robotReach,
            maxX = fieldExtent,
            minY = -fieldExtent,
            maxY = fieldExtent
        )
        ShootingZones.CLOSEST -> error("Closest zone should be expanded before computing bounds")
    }
}

private fun DecodeRobot.bestPlanInZone(
    shootingZone: ShootingZones,
    startPosition: Position,
    protectedZones: List<Location>,
    stayInAllianceHalf: Boolean
): ShootingApproachPlan? {
    val bounds = searchBoundsFor(shootingZone)
    val candidates = mutableListOf(startPosition.location())
    candidates += generateSearchGrid(bounds, ShootingZoneOptimizationConfig.coarseSearchStep).toList()

    val coarseBest = candidates.asSequence()
        .mapNotNull { evaluateShootingCandidate(shootingZone, startPosition, it, protectedZones, stayInAllianceHalf) }
        .minByOrNull { it.estimatedTime }
        ?: return null

    val fineBounds = SearchBounds(
        minX = coarseBest.target.x - ShootingZoneOptimizationConfig.fineSearchRadius,
        maxX = coarseBest.target.x + ShootingZoneOptimizationConfig.fineSearchRadius,
        minY = coarseBest.target.y - ShootingZoneOptimizationConfig.fineSearchRadius,
        maxY = coarseBest.target.y + ShootingZoneOptimizationConfig.fineSearchRadius
    )

    return generateSearchGrid(fineBounds, ShootingZoneOptimizationConfig.fineSearchStep)
        .plus(sequenceOf(coarseBest.target.location()))
        .mapNotNull { evaluateShootingCandidate(shootingZone, startPosition, it, protectedZones, stayInAllianceHalf) }
        .minByOrNull { it.estimatedTime }
        ?: coarseBest
}

internal fun DecodeRobot.planShootingApproach(
    shootingZone: ShootingZones,
    startPosition: Position = currentPosition,
    protectedZones: List<Location> = emptyList(),
    stayInAllianceHalf: Boolean = false
): ShootingApproachPlan {
    val zones = when (shootingZone) {
        ShootingZones.CLOSEST -> listOf(ShootingZones.FRONT, ShootingZones.BACK)
        else -> listOf(shootingZone)
    }

    return zones.asSequence()
        .mapNotNull { bestPlanInZone(it, startPosition, protectedZones, stayInAllianceHalf) }
        .minByOrNull { it.estimatedTime }
        ?: error("Unable to find a valid shooting pose for $shootingZone")
}

fun DecodeRobot.inShootingZone(shootingZone: ShootingZones = ShootingZones.CLOSEST): Boolean {
    val l = getPart(RobotLocations)
    val points = listOf(
        l.rf_corner,
        l.lf_corner,
        l.rb_corner,
        l.lb_corner,
        l.rf_corner.between(l.lf_corner),
        l.lf_corner.between(l.lb_corner),
        l.lb_corner.between(l.rb_corner),
        l.rb_corner.between(l.rf_corner)
    )

    return points.any { pointInShootingZone(it, shootingZone) }
}

fun DecodeRobot.clearForShooting(): Boolean{
    fun headingIsAtGoal(): Boolean{
        val sideDistanceDeviation = 15.cm
        val distanceToGoal = goalDistance()
        val maxHeadingDeviation = atan2(sideDistanceDeviation, distanceToGoal)
        val headingToGoal = headingToGoal()
        return shooterAngle in headingToGoal - maxHeadingDeviation .. headingToGoal + maxHeadingDeviation
    }

    fun flySpeedIsCorrect(): Boolean{
        return true
    }
    // TODO: add a check if the speed if the flywheel is good
    return inShootingZone() && headingIsAtGoal() && flySpeedIsCorrect()
}


fun DecodeRobot.closestPointInShootingZone(
    shootingZone: ShootingZones,
    position: Position = currentPosition,
    protectedZones: List<Location> = emptyList(),
    stayInAllianceHalf: Boolean = false
): Location {
    return planShootingApproach(shootingZone, position, protectedZones, stayInAllianceHalf).target.location()
}

fun DecodeRobot.closestPointInShootingZone(
    shootingZone: ShootingZones,
    position: Location,
    protectedZones: List<Location> = emptyList(),
    stayInAllianceHalf: Boolean = false
): Location {
    return closestPointInShootingZone(
        shootingZone = shootingZone,
        position = position.withHeading(currentPosition.heading),
        protectedZones = protectedZones,
        stayInAllianceHalf = stayInAllianceHalf
    )
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
