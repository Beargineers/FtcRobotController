package org.beargineers.platform.decode

import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.beargineers.platform.Location
import org.beargineers.platform.Position
import org.beargineers.platform.RobotDimensions
import org.beargineers.platform.Waypoint
import org.beargineers.platform.abs
import org.beargineers.platform.atan2
import org.beargineers.platform.buildPath
import org.beargineers.platform.cancelWhen
import org.beargineers.platform.cm
import org.beargineers.platform.degrees
import org.beargineers.platform.drivePath
import org.beargineers.platform.driveTo
import org.beargineers.platform.inch
import org.beargineers.platform.nextTick
import org.beargineers.platform.pathTo
import kotlin.time.Duration.Companion.seconds

suspend fun DecodeRobot.followPathAndShoot(waypoints: List<Waypoint>, applyMirroring: Boolean) {
    coroutineScope {
        launch {
            prepareForShooting()
        }

        val waypoints = waypoints.withIndex().map { (i, waypoint) ->
            if (i == waypoints.lastIndex) waypoint.copy(
                positionTolerance = 3.cm
            ) else waypoint
        }
        drivePath(waypoints, applyMirroring)

        shoot(true)
    }
}

suspend fun DecodeRobot.pushAllianceBot() {
    drivePath(pathTo(
        currentPosition.shift(0.cm, 30.cm),
        positionTolerance = 4.cm,
        headingTolerance = 3.degrees
    ), true)
}

suspend fun DecodeRobot.openRamp() {
    drivePath(openRampPath(), true)
    delay(AutoPositions.OPEN_RAMP_WAIT_TIME.seconds)

    if (hasVision) {
        driveTo(Locations.COLLECT_FROM_OPEN_RAMP_APPROACH, applyMirroring = true)
        collectArtifactsInView(true) {
            it.x > 0.cm && it.x < spikeStart(2).x - (RobotDimensions.ROBOT_WIDTH / 2 + 5.inch)
        }
    }
    else {
        drivePath(buildPath {
            with(Locations) {
                addWaypoint(COLLECT_FROM_OPEN_RAMP_APPROACH)
                addWaypoint(COLLECT_FROM_OPEN_RAMP)
            }
        }, applyMirroring = true)
    }
}

suspend fun DecodeRobot.openRampAndCollect() {
    val path = openRampCollectPath()
    drivePath(path.take(1), true)
    drivePath(path.drop(1), true)

    cancelWhen({artifactsCount >= 3}) {
        delay(AutoPositions.COLLECT_FROM_RAMP_WAIT_TIME.seconds)
    }
}

suspend fun DecodeRobot.collectArtifactsInView(strafe: Boolean, filter: (Location) -> Boolean = {true}) {
    intakeMode = IntakeMode.ON

    var targetLocation = intakeTarget(filter) ?: return
    coroutineScope {
        outer@while (true) {
            val job = launch {
                val targetPosition = if (strafe) {
                    targetLocation.withHeading(currentPosition.heading)
                } else {
                    val dx = targetLocation.x - currentPosition.x
                    val dy = targetLocation.y - currentPosition.y
                    targetLocation.withHeading(atan2(dy, dx))
                }

                driveTo(targetPosition, applyMirroring = false)
            }

            while (true) {
                nextTick()
                val nextTarget = intakeTarget(filter)
                if (nextTarget != null) {
                    targetLocation = nextTarget
                    job.cancel()
                    break
                } else if (!job.isActive) {
                    break@outer
                }
            }
        }
    }
}

suspend fun DecodeRobot.shootInitialLoad(launchPose: Position) {
    followPathAndShoot(pathTo(launchPose, Locations.INITIAL_SHOT_SPEED), true)
}

suspend fun DecodeRobot.goToShootingZoneAndShoot(
    shootingZone: ShootingZones,
    protectedZones: List<Location> = emptyList(),
    stayInAllianceHalf: Boolean = false
) {
    val plan = planShootingApproach(
        shootingZone = shootingZone,
        protectedZones = protectedZones,
        stayInAllianceHalf = stayInAllianceHalf
    )
    val waypoints = buildPath {
        for (waypoint in plan.approachWaypoints) {
            addWaypoint(waypoint)
        }
        addWaypoint(plan.target)
    }

    followPathAndShoot(waypoints, false)
}

suspend fun DecodeRobot.park() {
    val parkCoords: Location = Locations.PARK.mirrorForAlliance(alliance)
    val heading = currentPosition.heading
    val squareAngles = listOf(-180.degrees, -90.degrees, 0.degrees, 90.degrees, 180.degrees)
    val parkHeading = squareAngles.minBy { abs((it - heading).normalize()) }
    driveTo(parkCoords.withHeading(parkHeading), applyMirroring = false)
}
