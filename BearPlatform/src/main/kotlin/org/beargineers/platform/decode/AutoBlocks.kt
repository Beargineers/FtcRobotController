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
import org.beargineers.platform.driveTo
import org.beargineers.platform.followPath
import org.beargineers.platform.tilePosition
import org.beargineers.platform.wait
import kotlin.time.Duration.Companion.seconds

@PhaseDsl
private fun PhaseBuilder<DecodeRobot>.scoopSpike(spike: Int) {

    seq("Scoop spike #$spike") {
        driveTo(robot.spikeStart(spike))
        driveTo(robot.spikeEnd(spike))

        // when collecting spike #2 we have to back off a bit up or otherwise we hit the ramp on the way to a shooting point
        if (spike == 2) {
            driveTo(robot.spikeEnd(spike))
        }
    }
}

@PhaseDsl
private fun PhaseBuilder<DecodeRobot>.scoopAndShoot(spike: Int, launchPose: Position) {
    seq("Scoop and shoot #$spike") {
        scoopSpike(spike)
        shootAt(launchPose)
    }
}

private fun PhaseBuilder<DecodeRobot>.shootAt(launchPose: Position) {
    //  followPath(listOf(spike.startPose, spike.endPose, spike.startPose, launchPose))
    driveTo(launchPose)
    doOnce {
        launch()
    }
    waitForShootingCompletion()
}

open class DecodeAutoStrategy(alliance: Alliance, val positions: String) :
    PhasedAutonomous<DecodeRobot>(alliance, {

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
    }) {

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
/*
    scoopSpike(spikes[0])
    openRamp()
    shootAt(launchPoint)
*/

    if (startingPoint.x.cm() > 0) {
        // Far shooting zone
        scoopAndShoot(1, launchPoint)
        scoopAndShoot(2, launchPoint)
        scoopAndShoot(3, launchPoint)
    }
    else {
        scoopAndShoot(3, launchPoint)
        scoopAndShoot(2, launchPoint)
        scoopAndShoot(1, launchPoint)
    }
}

fun PhaseBuilder<DecodeRobot>.openRamp() {
    with(robot.locations) {
        drive(listOf(
            Waypoint(OPEN_RAMP_APPROACH),
            Waypoint(OPEN_RAMP, OPEN_RAMP_SPEED)
        ))
        wait(1.seconds)
    }
}

@PhaseDsl
private fun PhaseBuilder<DecodeRobot>.shootInitialLoad(launchPose: Position) {
    driveTo(launchPose)
    doOnce {
        launch()
    }
    waitForShootingCompletion()
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

        if (currentPosition.distanceTo(locations.OPEN_RAMP) < 10.cm) {
            waypoints.add(Waypoint(currentPosition + RelativePosition(-15.cm, 0.cm, 0.degrees)))
        }

        val targetLocation = closestPointInShootingZone(shootingZone)
        waypoints.add(Waypoint(targetLocation.withHeading(headingToGoalFrom(targetLocation))))
    }

    action {
        followPath(waypoints)
    }

    doOnce {
        launch()
    }
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
