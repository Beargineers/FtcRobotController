package org.beargineers.platform.decode

import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.beargineers.platform.Location
import org.beargineers.platform.atan2
import org.beargineers.platform.buildPath
import org.beargineers.platform.cancelWhen
import org.beargineers.platform.cm
import org.beargineers.platform.degrees
import org.beargineers.platform.drivePath
import org.beargineers.platform.driveTo
import org.beargineers.platform.nextTick
import org.beargineers.platform.pathTo
import org.beargineers.platform.withName
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

suspend fun DecodeRobot.pushAllianceBot() {
    withName("Push alliance bot") {
        drivePath(pathTo(
            currentPosition.shift(0.cm, 30.cm),
            positionTolerance = 4.cm,
            headingTolerance = 3.degrees
        ), true)
    }
}

suspend fun DecodeRobot.openRamp() {
    withName("openRamp") {
        drivePath(openRampPath(), true)
        delay(AutoPositions.OPEN_RAMP_WAIT_TIME.seconds)
    }
}

suspend fun DecodeRobot.openRampAndCollect() {
    withName("openRampAndCollect") {
        val path = openRampCollectPath()
        drivePath(path.take(1), true)
        drivePath(path.drop(1), true)

        delay(100.milliseconds)

        intakeMode = IntakeMode.ON
        drivePath(buildPath {
            with(Locations) {
                addWaypoint(COLLECT_FROM_OPEN_RAMP_APPROACH)
                addWaypoint(COLLECT_FROM_OPEN_RAMP)
            }
        }, applyMirroring = true)

        cancelWhen({artifactsCount >= 3}) {
            delay(AutoPositions.COLLECT_FROM_RAMP_WAIT_TIME.seconds)
        }
    }
}

suspend fun DecodeRobot.collectArtifactsInView(strafe: Boolean, filter: (Location) -> Boolean = {true}) {
    intakeMode = IntakeMode.ON

    var targetLocation = intakeTarget(filter) ?: return
    withName("collectArtifactsInView") {
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
}

suspend fun DecodeRobot.goToShootingZoneAndShoot(
    shootingZone: ShootingZones,
    protectedZones: List<Location> = emptyList(),
    stayInAllianceHalf: Boolean = false
) {
    withName("goToShootingZoneAndShoot") {
        val plan = planShootingApproach(
            startPosition = currentPosition,
            shootingZone = shootingZone,
            protectedZones = protectedZones,
            stayInAllianceHalf = stayInAllianceHalf
        )

        followPathAndShoot(plan, false)
    }
}
