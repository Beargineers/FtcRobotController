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
import org.beargineers.platform.cm
import org.beargineers.platform.degrees
import org.beargineers.platform.doWhile
import org.beargineers.platform.drivePath
import org.beargineers.platform.driveTo
import org.beargineers.platform.followPath
import org.beargineers.platform.pathTo
import org.beargineers.platform.times
import kotlin.math.sign
import kotlin.time.Duration.Companion.seconds

suspend fun DecodeRobot.followPathAndShoot(waypoints: List<Waypoint>) {
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
        drivePath(waypoints)
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

suspend fun DecodeRobot.scoopAndShoot(spike: Int, launchPose: Position) {
    followPathAndShoot(scoopSpikePath(spike) + pathTo(launchPose))
}

suspend fun DecodeRobot.scoopFromBoxAndShoot(launchPose: Position) {
    doWhile({ artifactsCount < 3}) {
        followPath(scoopBoxPath(0.cm) + scoopBoxPath(20.cm) + scoopBoxPath(30.cm))
    }

    followPathAndShoot(pathTo(launchPose))
}

suspend fun DecodeRobot.pushAllianceBot(startingPoint: Position) {
    drivePath(pathTo(
        startingPoint.shift(0.cm, startingPoint.y.cm().sign * 30.cm),
        positionTolerance = 4.cm,
        headingTolerance = 3.degrees
    ))
}

suspend fun DecodeRobot.openRamp() {
    drivePath(openRampPath())
    delay(AutoPositions.OPEN_RAMP_WAIT_TIME.seconds)
    drivePath(pathTo(locations.COLLECT_FROM_OPEN_RAMP))
}

suspend fun DecodeRobot.openRampAndCollect() {
    val path = openRampCollectPath()
    drivePath(path.take(1))
    drivePath(path.drop(1))

    doWhile({artifactsCount < 3}) {
        delay(AutoPositions.COLLECT_FROM_RAMP_WAIT_TIME.seconds)
    }
}

suspend fun DecodeRobot.shootInitialLoad(launchPose: Position) {
    followPathAndShoot(pathTo(launchPose, locations.INITIAL_SHOT_SPEED))
}

suspend fun DecodeRobot.goToShootingZoneAndShoot(shootingZone: ShootingZones) {
    val waypoints = buildPath {
        if (currentPosition.distanceTo(locations.OPEN_RAMP_COLLECT) < 10.cm ||
            currentPosition.distanceTo(locations.OPEN_RAMP) < 10.cm) {
            addWaypoint(currentPosition + RobotCentricPosition(-15.cm, 0.cm, 0.degrees))
        }

        val targetLocation = closestPointInShootingZone(shootingZone)
        addWaypoint(targetLocation.withHeading(headingToGoalFrom(targetLocation)))
    }

    followPathAndShoot(waypoints)
}

suspend fun DecodeRobot.park() {
    val parkCoords: Location = locations.PARK
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
