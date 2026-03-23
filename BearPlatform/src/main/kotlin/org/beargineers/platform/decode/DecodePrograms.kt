package org.beargineers.platform.decode

import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.beargineers.platform.Location
import org.beargineers.platform.Position
import org.beargineers.platform.RobotCentricPosition
import org.beargineers.platform.Waypoint
import org.beargineers.platform.abs
import org.beargineers.platform.buildPath
import org.beargineers.platform.cancelWhen
import org.beargineers.platform.cm
import org.beargineers.platform.degrees
import org.beargineers.platform.drivePath
import org.beargineers.platform.driveTo
import org.beargineers.platform.max
import org.beargineers.platform.min
import org.beargineers.platform.pathTo
import kotlin.time.Duration.Companion.seconds

suspend fun DecodeRobot.followPathAndShoot(waypoints: List<Waypoint>, applyMirroring: Boolean = true) {
    coroutineScope {
        val prepare = launch {
            delay(0.2.seconds)
            intakeMode(IntakeMode.ON)
            delay(0.5.seconds)
            getReadyForShoot()
        }

        val waypoints = waypoints.withIndex().map { (i, waypoint) ->
            if (i == waypoints.lastIndex) waypoint.copy(
                positionTolerance = 3.cm
            ) else waypoint
        }
        drivePath(waypoints, applyMirroring)
        prepare.cancel()
    }

    shoot()
}

suspend fun DecodeRobot.shoot() {
    launch()
    while (isShooting()) {
        opMode.loop.nextTick()
    }
}

suspend fun DecodeRobot.pushAllianceBot(startingPoint: Position) {
    drivePath(pathTo(
        startingPoint.shift(0.cm, 30.cm),
        positionTolerance = 4.cm,
        headingTolerance = 3.degrees
    ))
}

suspend fun DecodeRobot.openRamp() {
    drivePath(openRampPath())
    delay(AutoPositions.OPEN_RAMP_WAIT_TIME.seconds)
    driveTo(Locations.COLLECT_FROM_OPEN_RAMP)
}

suspend fun DecodeRobot.openRampAndCollect() {
    val path = openRampCollectPath()
    drivePath(path.take(1))
    drivePath(path.drop(1))

    cancelWhen({artifactsCount >= 3}) {
        delay(AutoPositions.COLLECT_FROM_RAMP_WAIT_TIME.seconds)
    }
}

suspend fun DecodeRobot.shootInitialLoad(launchPose: Position) {
    followPathAndShoot(pathTo(launchPose, Locations.INITIAL_SHOT_SPEED))
}

suspend fun DecodeRobot.goToShootingZoneAndShoot(shootingZone: ShootingZones, protectedZones: List<Location> = emptyList()) {
    val waypoints = buildPath {
        var maxBackoff = 0.cm
        if (currentPosition.distanceTo(Locations.OPEN_RAMP_COLLECT) < 10.cm ||
            currentPosition.distanceTo(Locations.OPEN_RAMP) < 10.cm) {
            maxBackoff = 15.cm
        }

        val targetLocation = closestPointInShootingZone(shootingZone)
        val targetHeading = if (hasTurret) currentPosition.heading else headingToGoalFrom(targetLocation)

        val minX = min(currentPosition.x, targetLocation.x)
        val maxX = max(currentPosition.x, targetLocation.x)
        for (zone in protectedZones) {
            if (zone.x in minX..maxX) {
                maxBackoff = max(maxBackoff, abs(zone.y) - abs(currentPosition.y))
            }
        }

        if (maxBackoff > 0.cm) {
            addWaypoint(currentPosition + RobotCentricPosition(-maxBackoff, 0.cm, 0.degrees))
        }

        addWaypoint(targetLocation.withHeading(targetHeading))
    }

    followPathAndShoot(waypoints, false)
}

suspend fun DecodeRobot.park() {
    val parkCoords: Location = Locations.PARK
    val heading = currentPosition.heading
    val squareAngles = listOf(-180.degrees, -90.degrees, 0.degrees, 90.degrees, 180.degrees)
    val parkHeading = squareAngles.minBy { abs(it - heading) }
    driveTo(parkCoords.withHeading(parkHeading))
}

suspend fun DecodeRobot.holdPositionLookAtGoal(location: Location) {
    while (true) {
        driveTo(location.withHeading(headingToGoalFrom(location)))
    }
}
