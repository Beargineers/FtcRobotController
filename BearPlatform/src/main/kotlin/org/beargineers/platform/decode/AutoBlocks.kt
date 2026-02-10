package org.beargineers.platform.decode

import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.AutonomousPhase
import org.beargineers.platform.Distance
import org.beargineers.platform.Location
import org.beargineers.platform.PhaseBuilder
import org.beargineers.platform.PhaseDsl
import org.beargineers.platform.PhasedAutonomous
import org.beargineers.platform.Position
import org.beargineers.platform.RobotCentricPosition
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
import org.beargineers.platform.times
import org.beargineers.platform.wait
import kotlin.math.sign
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
    val COLLECT_FROM_RAMP_WAIT_TIME by config(1.5)
}

fun DecodeRobot.scoopSpikePath(spike: Int): List<Waypoint> {
    return buildPath {
        addWaypoint(spikeStart(spike))
        addWaypoint(spikeEnd(spike), locations.SPIKE_SCOOPING_SPEED)
    }
}

fun DecodeRobot.scoopBoxPath(shift: Distance): List<Waypoint> {
    return buildPath {
        addWaypoint(AutoPositions.BOX_APPROACH.shift(-shift, 0.cm).rotate(if (shift == 0.cm) -20.degrees else 0.degrees).mirrorForAlliance(alliance))
        addWaypoint(AutoPositions.BOX_SCOOP.shift(-shift, 0.cm).rotate(if (shift == 0.cm) -20.degrees else 0.degrees).mirrorForAlliance(alliance), AutoPositions.BOX_SCOOP_SPEED)
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
                followPath(scoopBoxPath(0.cm) + scoopBoxPath(20.cm) + scoopBoxPath(30.cm))
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

abstract class ProgrammedAuto() : PhasedAutonomous<DecodeRobot>() {
    abstract val program: String

    private var operatingIn: Char = 'N'

    override fun PhaseBuilder<DecodeRobot>.phases() {
        doWhile("AUTO") {
            condition {
                opMode.elapsedTime.seconds() < 29.5
            }

            looping {
                interpretProgram()
            }

            then {
                doOnce {
                    intakeMode(IntakeMode.OFF)
                    enableFlywheel(false)
                }
                action {
                    driveToTarget(when (operatingIn) {
                        'F' -> locations.OPEN_RAMP_APPROACH
                        'B' -> AutoPositions.BOX_APPROACH.mirrorForAlliance(alliance)
                        else -> error("Unknown operating in: $operatingIn")
                    })
                }
            }
        }
    }

    private fun PhaseBuilder<DecodeRobot>.interpretProgram() {
        val startingPoint = when(program.first()) {
            'F' -> AutoPositions.NORTH_START
            'B' -> AutoPositions.SOUTH_START
            else -> error("Program should start from either F or B to indicate starting position. Actual symbol: ${program.first()}")
        }.mirrorForAlliance(alliance)

        var shootingPoint = Position.zero()
        val path = mutableListOf<Waypoint>()
        var initialLoadReleased = false
        val collectedSet = mutableSetOf<Char>()
        var lastKnownPosition = Position.zero()

        fun pathToShooting(): List<Waypoint> {
            return buildList {
                var addedBackPath = false
                if (operatingIn == 'F') {
                    if ('1' !in collectedSet && lastKnownPosition.x > robot.spikeStart(1).x ||
                        '2' !in collectedSet && lastKnownPosition.x > robot.spikeStart(2).x ||
                        '3' !in collectedSet && lastKnownPosition.x > robot.spikeStart(3).x) {
                        addAll(pathTo(lastKnownPosition.copy(y = shootingPoint.y)))
                        addedBackPath = true
                    }
                }
                else {
                    if ('1' !in collectedSet && lastKnownPosition.x < robot.spikeStart(1).x ||
                        '2' !in collectedSet && lastKnownPosition.x < robot.spikeStart(2).x ||
                        '3' !in collectedSet && lastKnownPosition.x < robot.spikeStart(3).x) {
                        addAll(pathTo(lastKnownPosition.copy(y = shootingPoint.y)))
                        addedBackPath = true
                    }
                }

                if (!addedBackPath && (lastKnownPosition.distanceTo(robot.locations.OPEN_RAMP_COLLECT) < 10.cm ||
                    lastKnownPosition.distanceTo(robot.locations.OPEN_RAMP) < 10.cm)) {
                    addAll(pathTo(lastKnownPosition + RobotCentricPosition(-15.cm, 0.cm, 0.degrees)))
                }

                addAll(pathTo(shootingPoint))
            }
        }

        fun shootIfNeeded() {
            if (!initialLoadReleased) {
                shootInitialLoad(shootingPoint)
                lastKnownPosition = shootingPoint
                initialLoadReleased = true

                if (path.isNotEmpty()) {
                    error("Path should be empty before we shoot initial load")
                }
            }

            if (path.isNotEmpty()) {
                path.addAll(pathToShooting())
                followPathAndShoot(path.toList())
                lastKnownPosition = path.last().target
                path.clear()
            }
        }

        fun followPathIfNeeded() {
            if (path.isNotEmpty()) {
                drive(path.toList())
                lastKnownPosition = path.last().target
                path.clear()
            }
        }

        assumeRobotPosition(startingPoint)
        lastKnownPosition = startingPoint

        doOnce {
            enableFlywheel(true)
        }

        for (c in program) {
            when (c) {
                'F' -> {
                    operatingIn = 'F'
                    shootingPoint = AutoPositions.NORTH_SHOOTING.mirrorForAlliance(alliance)
                }

                'B' -> {
                    operatingIn = 'B'
                    shootingPoint = AutoPositions.SOUTH_SHOOTING.mirrorForAlliance(alliance)
                }

                '0' -> {
                    shootIfNeeded()
                    scoopFromBoxAndShoot(shootingPoint)
                    lastKnownPosition = shootingPoint
                    collectedSet += '0'
                }

                '1' -> {
                    shootIfNeeded()
                    path.addAll(robot.scoopSpikePath(1))
                    lastKnownPosition = path.last().target
                    collectedSet += '1'
                }

                '2' -> {
                    shootIfNeeded()
                    path.addAll(robot.scoopSpikePath(2))
                    lastKnownPosition = path.last().target
                    collectedSet += '2'
                }

                '3' -> {
                    shootIfNeeded()
                    path.addAll(robot.scoopSpikePath(3))
                    lastKnownPosition = path.last().target
                    collectedSet += '3'
                }

                '4' -> {
                    shootIfNeeded()
                    val farApproach =
                        robot.locations.OPEN_RAMP_COLLECT_APPROACH.copy(y = robot.spikeStart(1).y)
                    drive(pathTo(farApproach))
                    openRampAndCollect()
                    lastKnownPosition = robot.locations.OPEN_RAMP_COLLECT
                    followPathAndShoot(pathToShooting())
                    collectedSet += '4'
                }

                '/' -> {
                    shootIfNeeded()
                }

                'R' -> {
                    val pathNotEmpty = path.isNotEmpty()
                    path.addAll(robot.openRampPath())
                    followPathIfNeeded()
                    wait(AutoPositions.OPEN_RAMP_WAIT_TIME.seconds)
                    if (pathNotEmpty) {
                        followPathAndShoot(pathToShooting())
                    }
                }

                'P' -> {
                    pushAllianceBot(startingPoint)
                }

                ' ' -> {
                    // Do nothing
                }

                else -> {
                    error("Unknown command symbol: $c")
                }
            }
        }

        shootIfNeeded()
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

fun PhaseBuilder<DecodeRobot>.pushAllianceBot(startingPoint: Position) {
    drive(pathTo(
        startingPoint.shift(0.cm, startingPoint.y.cm().sign * 30.cm),
        positionTolerance = 4.cm,
        headingTolerance = 3.degrees
    ))
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

    doWhile("Wait for the artifacts") {
        condition { artifactsCount < 3 }
        looping {
            wait(AutoPositions.COLLECT_FROM_RAMP_WAIT_TIME.seconds)
        }
        then {
            // Do nothing
        }
    }
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
                addWaypoint(currentPosition + RobotCentricPosition(-15.cm, 0.cm, 0.degrees))
            }

            val targetLocation = closestPointInShootingZone(shootingZone)
            addWaypoint(targetLocation.withHeading(headingToGoalFrom(targetLocation)))
        })

    }

    followPathAndShoot(waypoints)
}

@PhaseDsl
fun PhaseBuilder<DecodeRobot>.followPathAndShoot(waypoints: List<Waypoint>) {
    seq("Follow path and prepare for shooting") {
        var followingPath = false

        doOnce {
            followingPath = true
        }

        par {
            doWhile("Prepare for shooting") {
                condition {
                    followingPath
                }

                looping {
                    wait(0.2.seconds)
                    doOnce {
                        intakeMode(IntakeMode.ON)
                    }
                    wait(0.5.seconds)
                    doOnce {
                        getReadyForShoot()
                    }
                }

                then {
                    // Do nothing
                }
            }

            action {
                val waypoints = waypoints.withIndex().map { (i, waypoint) ->
                    if (i == waypoints.lastIndex) waypoint.copy(
                        positionTolerance = 3.cm
                    ) else waypoint
                }
                followingPath = followPath(waypoints)
                followingPath
            }
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
