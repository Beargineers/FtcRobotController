package org.beargineers.platform.decode

import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.Alliance
import org.beargineers.platform.AutonomousPhase
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
import org.beargineers.platform.cm
import org.beargineers.platform.cursorLocation
import org.beargineers.platform.degrees
import org.beargineers.platform.doOnce
import org.beargineers.platform.drive
import org.beargineers.platform.followPath
import org.beargineers.platform.tilePosition
import org.beargineers.platform.wait
import kotlin.time.Duration.Companion.seconds

fun DecodeRobot.scoopSpikePath(spike: Int): List<Waypoint> {
    return buildList {
        add(Waypoint(spikeStart(spike)))
        add(Waypoint(spikeEnd(spike), locations.SPIKE_SCOOPING_SPEED))

        if (spike == 2) {
            add(Waypoint(spikeStart(spike)))
        }
    }
}

@PhaseDsl
fun PhaseBuilder<DecodeRobot>.scoopAndShoot(spike: Int, launchPose: Position) {
    seq("Scoop and shoot #$spike") {
        followPathAndShoot(robot.scoopSpikePath(spike) + Waypoint(launchPose))
    }
}

@PhaseDsl
fun PhaseBuilder<DecodeRobot>.scoopFromBoxAndShoot(launchPose: Position) {
    seq("Scoop from BOX and shoot") {
        val boxCollectionPath = robot.scoopSpikePath(0).map {
            Waypoint(
                it.target.shift(11.cm, if (robot.alliance == Alliance.RED) 5.cm else -5.cm),
                it.speed,
                it.onArrival
            )
        }
        followPathAndShoot(boxCollectionPath + boxCollectionPath + boxCollectionPath +  Waypoint(launchPose))
    }
}

private fun PhaseBuilder<DecodeRobot>.shoot() {
    doOnce {
        launch()
    }
    waitForShootingCompletion()
}

open class DecodeAutoStrategy(alliance: Alliance, val positions: String) : PhasedAutonomous<DecodeRobot>(alliance) {
    override fun PhaseBuilder<DecodeRobot>.phases() {
        val (startingPoint, shootingPoint) = positions.split(",").map { it.trim() }

        doWhile("AUTO") {
            condition {
                opMode.elapsedTime.seconds() < 29
            }

            looping {
                autoStrategy(tilePosition(startingPoint), tilePosition(shootingPoint))
            }

            then {
                doOnce {
                    intakeMode(IntakeMode.OFF)
                    enableFlywheel(false)
                }
                action {
                    driveToTarget(locations.OPEN_RAMP_APPROACH)
                }
            }
        }
    }

    override fun bearInit() {
        super.bearInit()

        telemetry.addLine("Ensure robot is in position: ${positions.takeWhile { it != ',' }}")
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
        followPathAndShoot(listOf(secondScoop.last(), Waypoint(launchPoint)))
        // Far shooting zone
        scoopAndShoot(3, launchPoint)
        scoopAndShoot(1, launchPoint)
    }
    else {
        // Far shooting zone
        repeat(5) {
            scoopFromBoxAndShoot(launchPoint)
        }
    }
}

fun DecodeRobot.openRampPath(): List<Waypoint> {
    return buildList {
        with(locations) {
            add(Waypoint(OPEN_RAMP_APPROACH))
            add(Waypoint(OPEN_RAMP, OPEN_RAMP_SPEED))
        }
    }
}

fun DecodeRobot.openRampCollectPath(): List<Waypoint> {
    return buildList {
        with(locations) {
            add(Waypoint(OPEN_RAMP_COLLECT_APPROACH))
            add(Waypoint(OPEN_RAMP_COLLECT, OPEN_RAMP_SPEED))
        }
    }
}

fun PhaseBuilder<DecodeRobot>.openRamp() {
    drive(robot.openRampPath())
    wait(1.seconds)
}

fun PhaseBuilder<DecodeRobot>.openRampAndCollect() {
    drive(robot.openRampCollectPath())
    wait(1.seconds)
}

@PhaseDsl
private fun PhaseBuilder<DecodeRobot>.shootInitialLoad(launchPose: Position) {
    followPathAndShoot(listOf(Waypoint(launchPose, robot.locations.INITIAL_SHOT_SPEED)))
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

        if (currentPosition.distanceTo(locations.OPEN_RAMP_COLLECT) < 10.cm ||
            currentPosition.distanceTo(locations.OPEN_RAMP) < 10.cm) {
            waypoints.add(Waypoint(currentPosition + RelativePosition(-15.cm, 0.cm, 0.degrees)))
        }

        val targetLocation = closestPointInShootingZone(shootingZone)
        waypoints.add(Waypoint(targetLocation.withHeading(headingToGoalFrom(targetLocation))))
    }

    followPathAndShoot(waypoints)
}

@PhaseDsl
private fun PhaseBuilder<DecodeRobot>.followPathAndShoot(waypoints: List<Waypoint>) {
    action {
        followPath(waypoints)
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
