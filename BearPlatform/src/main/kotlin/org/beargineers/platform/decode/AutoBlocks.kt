package org.beargineers.platform.decode

import kotlinx.coroutines.delay
import org.beargineers.platform.Distance
import org.beargineers.platform.Location
import org.beargineers.platform.Position
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.Waypoint
import org.beargineers.platform.buildPath
import org.beargineers.platform.cancelWhen
import org.beargineers.platform.cm
import org.beargineers.platform.config
import org.beargineers.platform.degrees
import org.beargineers.platform.drivePath
import org.beargineers.platform.driveTo
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

fun scoopSpikePath(spike: Int): List<Waypoint> {
    return buildPath {
        addWaypoint(spikeStart(spike))
        addWaypoint(spikeEnd(spike), Locations.SPIKE_SCOOPING_SPEED)
    }
}

fun scoopBoxPath(shift: Distance): List<Waypoint> {
    return buildPath {
        addWaypoint(AutoPositions.BOX_APPROACH.shift(-shift, 0.cm).rotate(if (shift == 0.cm) -20.degrees else 0.degrees))
        addWaypoint(AutoPositions.BOX_SCOOP.shift(-shift, 0.cm).rotate(if (shift == 0.cm) -20.degrees else 0.degrees), AutoPositions.BOX_SCOOP_SPEED)
    }
}


abstract class ProgrammedAuto : RobotOpMode<DecodeRobot>() {
    abstract val program: String

    override suspend fun DecodeRobot.autoProgram() {
        cancelWhen({ robot.opMode.elapsedTime.seconds() > 29.5}) {
            interpretProgram(program)
        }

        intakeMode(IntakeMode.OFF)
        enableFlywheel(false)

        val operatingIn = program.last {it == 'F' || it == 'B'}

        driveTo(when (operatingIn) {
            'F' -> Locations.OPEN_RAMP_APPROACH
            'B' -> AutoPositions.BOX_APPROACH
            else -> error("Unknown operating in: $operatingIn")
        })
    }
}

suspend fun DecodeRobot.interpretProgram(program: String) {
    val (startingPoint, initialShootingPoint) = when (program.first()) {
        'F' -> Pair(AutoPositions.NORTH_START, AutoPositions.NORTH_SHOOTING)
        'B' -> Pair(AutoPositions.SOUTH_START, AutoPositions.SOUTH_SHOOTING)
        else -> error("Program should start from either F or B to indicate starting position. Actual symbol: ${program.first()}")
    }

    var operatingIn = 'N'
    val collectedSet = mutableSetOf<Char>()
    var hasLoad = false

    fun protectedZones()= buildList {
        add(Location(0.cm, 30.cm)) // For the ramp handle
        if ('1' !in collectedSet) add(spikeStart(1).location())
        if ('2' !in collectedSet) add(spikeStart(2).location())
        if ('3' !in collectedSet) add(spikeStart(3).location())
    }

    suspend fun goAndShootIfHasLoad() {
        if (hasLoad) {
            goToShootingZoneAndShoot(
                if (operatingIn == 'F') ShootingZones.FRONT else ShootingZones.BACK,
                protectedZones()
            )
            hasLoad = false
        }
    }

    suspend fun collect(from: Char, path: List<Waypoint>) {
        goAndShootIfHasLoad()
        cancelWhen({artifactsCount >= 3}) {
            drivePath(path)
        }
        hasLoad = true
        collectedSet += from
    }

    assumePosition(startingPoint.mirrorForAlliance(alliance), 0.degrees)
    enableFlywheel(true)
    shootInitialLoad(initialShootingPoint)

    for (c in program) {
        when (c) {
            'F' -> operatingIn = 'F'
            'B' -> operatingIn = 'B'

            '/' -> goAndShootIfHasLoad()
            'P' -> pushAllianceBot(startingPoint)

            '0' -> collect('0', scoopBoxPath(0.cm) + scoopBoxPath(20.cm) + scoopBoxPath(30.cm))
            '1' -> collect('1', scoopSpikePath(1))
            '2' -> collect('2', scoopSpikePath(2))
            '3' -> collect('3', scoopSpikePath(3))

            '4' -> {
                goAndShootIfHasLoad()
                val farApproach =
                    Locations.OPEN_RAMP_COLLECT_APPROACH.copy(y = spikeStart(1).y)
                driveTo(farApproach)
                openRampAndCollect()
                collectedSet += '4'
                hasLoad = true
                goAndShootIfHasLoad()
            }

            'R' -> {
                drivePath(openRampPath())
                delay(AutoPositions.OPEN_RAMP_WAIT_TIME.seconds)
            }

            ' ' -> { /* Do nothing */ }
            else -> error("Unknown command symbol: $c")
        }
    }

    goAndShootIfHasLoad()
}

fun openRampPath(): List<Waypoint> {
    return buildPath {
        with(Locations) {
            addWaypoint(OPEN_RAMP_APPROACH, positionTolerance = 2.cm, headingTolerance = 2.degrees)
            addWaypoint(OPEN_RAMP, OPEN_RAMP_SPEED, positionTolerance = 1.cm, headingTolerance = 1.degrees)
        }
    }
}

fun openRampCollectPath(): List<Waypoint> {
    return buildPath {
        with(Locations) {
            addWaypoint(OPEN_RAMP_COLLECT_APPROACH)
            addWaypoint(OPEN_RAMP_COLLECT, OPEN_RAMP_SPEED)
        }
    }
}
