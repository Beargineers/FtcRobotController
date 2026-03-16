package org.beargineers.platform.decode

import kotlinx.coroutines.delay
import org.beargineers.platform.Distance
import org.beargineers.platform.Position
import org.beargineers.platform.RobotCentricPosition
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.Waypoint
import org.beargineers.platform.buildPath
import org.beargineers.platform.cm
import org.beargineers.platform.config
import org.beargineers.platform.degrees
import org.beargineers.platform.doWhile
import org.beargineers.platform.drivePath
import org.beargineers.platform.driveTo
import org.beargineers.platform.pathTo
import org.beargineers.platform.tilePosition
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


abstract class ProgrammedAuto : RobotOpMode<DecodeRobot>() {
    abstract val program: String

    private var operatingIn: Char = 'N'

    override suspend fun DecodeRobot.autoProgram() {
        doWhile({ robot.opMode.elapsedTime.seconds() < 29.5}) {
            interpretProgram()
        }

        intakeMode(IntakeMode.OFF)
        enableFlywheel(false)

        driveTo(when (operatingIn) {
            'F' -> locations.OPEN_RAMP_APPROACH
            'B' -> AutoPositions.BOX_APPROACH.mirrorForAlliance(alliance)
            else -> error("Unknown operating in: $operatingIn")
        })
    }

    private suspend fun DecodeRobot.interpretProgram() {
        val startingPoint = when(program.first()) {
            'F' -> AutoPositions.NORTH_START
            'B' -> AutoPositions.SOUTH_START
            else -> error("Program should start from either F or B to indicate starting position. Actual symbol: ${program.first()}")
        }.mirrorForAlliance(alliance)

        var shootingPoint = Position.zero()
        val path = mutableListOf<Waypoint>()
        var initialLoadReleased = false
        val collectedSet = mutableSetOf<Char>()

        fun pathToShooting(): List<Waypoint> {
            val cp = currentPosition
            return buildList {
                var addedBackPath = false
                if (operatingIn == 'F') {
                    if ('1' !in collectedSet && cp.x > robot.spikeStart(1).x ||
                        '2' !in collectedSet && cp.x > robot.spikeStart(2).x ||
                        '3' !in collectedSet && cp.x > robot.spikeStart(3).x) {
                        addAll(pathTo(cp.copy(y = shootingPoint.y)))
                        addedBackPath = true
                    }
                }
                else {
                    if ('1' !in collectedSet && cp.x < robot.spikeStart(1).x ||
                        '2' !in collectedSet && cp.x < robot.spikeStart(2).x ||
                        '3' !in collectedSet && cp.x < robot.spikeStart(3).x) {
                        addAll(pathTo(cp.copy(y = shootingPoint.y)))
                        addedBackPath = true
                    }
                }

                if (!addedBackPath && (cp.distanceTo(robot.locations.OPEN_RAMP_COLLECT) < 10.cm ||
                    cp.distanceTo(robot.locations.OPEN_RAMP) < 10.cm)) {
                    addAll(pathTo(cp + RobotCentricPosition(-15.cm, 0.cm, 0.degrees)))
                }

                addAll(pathTo(shootingPoint))
            }
        }

        suspend fun shootIfNeeded() {
            if (!initialLoadReleased) {
                shootInitialLoad(shootingPoint)
                initialLoadReleased = true

                if (path.isNotEmpty()) {
                    error("Path should be empty before we shoot initial load")
                }
            }

            if (path.isNotEmpty()) {
                path.addAll(pathToShooting())
                followPathAndShoot(path.toList())
                path.clear()
            }
        }

        suspend fun followPathIfNeeded() {
            if (path.isNotEmpty()) {
                drivePath(path.toList())
                path.clear()
            }
        }

        assumePosition(startingPoint)
        enableFlywheel(true)

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
                    collectedSet += '0'
                }

                '1' -> {
                    shootIfNeeded()
                    path.addAll(robot.scoopSpikePath(1))
                    collectedSet += '1'
                }

                '2' -> {
                    shootIfNeeded()
                    path.addAll(robot.scoopSpikePath(2))
                    collectedSet += '2'
                }

                '3' -> {
                    shootIfNeeded()
                    path.addAll(robot.scoopSpikePath(3))
                    collectedSet += '3'
                }

                '4' -> {
                    shootIfNeeded()
                    val farApproach =
                        robot.locations.OPEN_RAMP_COLLECT_APPROACH.copy(y = robot.spikeStart(1).y)
                    drivePath(pathTo(farApproach))
                    openRampAndCollect()
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
                    delay(AutoPositions.OPEN_RAMP_WAIT_TIME.seconds)
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

