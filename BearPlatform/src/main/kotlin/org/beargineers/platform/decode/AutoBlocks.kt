package org.beargineers.platform.decode

import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.Alliance
import org.beargineers.platform.AutonomousPhase
import org.beargineers.platform.Distance
import org.beargineers.platform.Location
import org.beargineers.platform.PhaseBuilder
import org.beargineers.platform.PhaseDsl
import org.beargineers.platform.PhasedAutonomous
import org.beargineers.platform.Position
import org.beargineers.platform.RelativePosition
import org.beargineers.platform.Waypoint
import org.beargineers.platform.abs
import org.beargineers.platform.action
import org.beargineers.platform.assumeRobotPosition
import org.beargineers.platform.buildPath
import org.beargineers.platform.cm
import org.beargineers.platform.config
import org.beargineers.platform.cursorLocation
import org.beargineers.platform.degrees
import org.beargineers.platform.doOnce
import org.beargineers.platform.drive
import org.beargineers.platform.followPath
import org.beargineers.platform.pathTo
import org.beargineers.platform.tilePosition
import org.beargineers.platform.wait
import kotlin.time.Duration.Companion.seconds

object AutoPositions {
    val NORTH_START by config(tilePosition("F6BL:+135"))
    val NORTH_SHOOTING by config(tilePosition("D4:+135"))
    val SOUTH_START by config(Position(160.cm,44.cm,180.degrees))
    val SOUTH_SHOOTING by config(tilePosition("D1:+160"))
    val BOX_APPROACH by config(tilePosition("D4:+135"))
    val BOX_SCOOP by config(tilePosition("D4:+135"))
    val BOX_SCOOP_SPEED by config(1.0)

    val OPEN_RAMP_WAIT_TIME by config(0.01)
}

fun DecodeRobot.scoopSpikePath(spike: Int): List<Waypoint> {
    return buildPath {
        addWaypoint(spikeStart(spike))
        addWaypoint(spikeEnd(spike), locations.SPIKE_SCOOPING_SPEED)

        if (spike == 2) {
            addWaypoint(spikeStart(spike))
        }
    }
}

fun DecodeRobot.scoopBoxPath(shift: Distance): List<Waypoint> {
    return buildPath {
        addWaypoint(AutoPositions.BOX_APPROACH.shift(-shift, 0.cm).mirrorForAlliance(alliance))
        addWaypoint(AutoPositions.BOX_SCOOP.shift(-shift, 0.cm).mirrorForAlliance(alliance), AutoPositions.BOX_SCOOP_SPEED)
    }
}



@PhaseDsl
fun PhaseBuilder<DecodeRobot>.scoopAndShoot(spike: Int, launchPose: Position) {
    seq("Scoop and shoot #$spike") {
        followPathAndShoot(robot.scoopSpikePath(spike) + pathTo(launchPose))
    }
}

@PhaseDsl
fun PhaseBuilder<DecodeRobot>.scoopFromBoxAndShoot(launchPose: Position) {
    doWhile("Scoop from BOX and shoot") {
        condition { artifactsCount < 3 }

        looping {
            action {
                followPath(scoopBoxPath(0.cm) + scoopBoxPath(20.cm) + scoopBoxPath(40.cm))
            }
        }

        then {
            followPathAndShoot(pathTo(launchPose))
        }
    }
}

private fun PhaseBuilder<DecodeRobot>.shoot() {
    doOnce {
        launch()
    }
    waitForShootingCompletion()
}


open class DecodeAutoStrategy(alliance: Alliance, val zone: ShootingZones) : PhasedAutonomous<DecodeRobot>(alliance) {
    val startingPoint = (if (zone == ShootingZones.FRONT) {
        AutoPositions.NORTH_START
    } else {
        AutoPositions.SOUTH_START
    }).mirrorForAlliance(alliance)

    val shootingPoint = (if (zone == ShootingZones.FRONT) {
        AutoPositions.NORTH_SHOOTING
    } else {
        AutoPositions.SOUTH_SHOOTING
    }).mirrorForAlliance(alliance)

    override fun PhaseBuilder<DecodeRobot>.phases() {
        doWhile("AUTO") {
            condition {
                opMode.elapsedTime.seconds() < 29
            }

            looping {
                autoStrategy(startingPoint, shootingPoint)
            }

            then {
                doOnce {
                    intakeMode(IntakeMode.OFF)
                    enableFlywheel(false)
                }
                action {
                    driveToTarget(if (zone == ShootingZones.FRONT) locations.OPEN_RAMP_APPROACH else AutoPositions.BOX_APPROACH.mirrorForAlliance(alliance))
                }
            }
        }
    }

    override fun bearInit() {
        super.bearInit()

        telemetry.addLine("Ensure robot is in position: $startingPoint")
    }
}

@PhaseDsl
private fun PhaseBuilder<DecodeRobot>.autoStrategy(startingPoint: Position, launchPoint: Position) {
    assumeRobotPosition(startingPoint)
    doOnce {
        enableFlywheel(true)
    }

    shootInitialLoad(launchPoint)

    if (launchPoint.x.cm() < 0) {
        // Near shooting zone
        val secondScoop = robot.scoopSpikePath(2)
        drive(secondScoop.take(2) + robot.openRampPath())
        wait(1.seconds)
        followPathAndShoot(secondScoop.drop(2) + pathTo(launchPoint))
        // Far shooting zone
        scoopAndShoot(3, launchPoint)
        scoopAndShoot(1, launchPoint)
    }
    else {
        scoopAndShoot(1, launchPoint)
        // Far shooting zone
        repeat(5) {
            scoopFromBoxAndShoot(launchPoint)
        }
    }
}

fun DecodeRobot.openRampPath(): List<Waypoint> {
    return buildPath {
        with(locations) {
            addWaypoint(OPEN_RAMP_APPROACH, positionTolerance = 2.cm, headingTolerance = 2.degrees)
            addWaypoint(OPEN_RAMP, OPEN_RAMP_SPEED, positionTolerance = 1.cm, headingTolerance = 1.degrees)
        }
    }
}

fun DecodeRobot.openRampCollectPath(): List<Waypoint> {
    return buildPath {
        with(locations) {
            addWaypoint(OPEN_RAMP_COLLECT_APPROACH)
            addWaypoint(OPEN_RAMP_COLLECT, OPEN_RAMP_SPEED)
        }
    }
}

fun PhaseBuilder<DecodeRobot>.openRamp() {
    drive(robot.openRampPath())
    wait(AutoPositions.OPEN_RAMP_WAIT_TIME.seconds)
    drive(pathTo(robot.locations.COLLECT_FROM_OPEN_RAMP))
}

fun PhaseBuilder<DecodeRobot>.openRampAndCollect() {
    val path = robot.openRampCollectPath()
    drive(path.take(1))
    drive(path.drop(1))
    wait(1.seconds)
}

@PhaseDsl
private fun PhaseBuilder<DecodeRobot>.shootInitialLoad(launchPose: Position) {
    followPathAndShoot(pathTo(launchPose, robot.locations.INITIAL_SHOT_SPEED))
}

class WaitForShootingCompletion() : AutonomousPhase<DecodeRobot> {
    override fun DecodeRobot.initPhase() {
    }

    override fun DecodeRobot.loopPhase(phaseTime: ElapsedTime): Boolean {
        return isShooting()
    }
}

@PhaseDsl
fun PhaseBuilder<DecodeRobot>.waitForShootingCompletion() {
    phase(WaitForShootingCompletion())
}

fun PhaseBuilder<DecodeRobot>.goToShootingZoneAndShoot(shootingZone: ShootingZones) {
    val waypoints = mutableListOf<Waypoint>()
    doOnce {
        waypoints.clear()

        waypoints.addAll(buildPath {
            if (currentPosition.distanceTo(locations.OPEN_RAMP_COLLECT) < 10.cm ||
                currentPosition.distanceTo(locations.OPEN_RAMP) < 10.cm) {
                addWaypoint(currentPosition + RelativePosition(-15.cm, 0.cm, 0.degrees))
            }

            val targetLocation = closestPointInShootingZone(shootingZone)
            addWaypoint(targetLocation.withHeading(headingToGoalFrom(targetLocation)))
        })

    }

    followPathAndShoot(waypoints)
}

@PhaseDsl
fun PhaseBuilder<DecodeRobot>.followPathAndShoot(waypoints: List<Waypoint>) {
    par("Follow path and shoot") {
        seq("Follow path") {
            wait(0.2.seconds)
            doOnce {
                intakeMode(IntakeMode.ON)
            }
            wait(0.5.seconds)
            doOnce {
                getReadyForShoot()
            }
        }

        action {
            val waypoints = waypoints.withIndex().map { (i, waypoint) ->
                    if (i == waypoints.lastIndex) waypoint.copy(
                        positionTolerance = 3.cm
                    ) else waypoint
                }
            followPath(waypoints)
        }
    }

    shoot()
}

fun PhaseBuilder<DecodeRobot>.goToCursorLocation(){
    action{
        driveToTarget(cursorLocation().withHeading(currentPosition.heading))
    }
}

fun PhaseBuilder<DecodeRobot>.park() {
    action {
        val parkCoords: Location = locations.PARK
        val heading = currentPosition.heading
        val squareAngles = listOf(-180.degrees, -90.degrees, 0.degrees, 90.degrees, 180.degrees)
        val parkHeading = squareAngles.minBy { abs(it - heading) }
        driveToTarget(parkCoords.withHeading(parkHeading))
    }
}

fun PhaseBuilder<DecodeRobot>.holdPositionLookAtGoal(location: Location) {
    action {
        driveToTarget(location.withHeading(headingToGoalFrom(location)))
        true // Keep this auto active until it is cancelled by touching gamepad controls
    }
}
