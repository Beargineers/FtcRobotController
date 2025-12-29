package org.beargineers.platform.decode

import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.Alliance
import org.beargineers.platform.AutonomousPhase
import org.beargineers.platform.BLUE_OPEN_GATE
import org.beargineers.platform.BLUE_OPEN_GATE_APPROACH
import org.beargineers.platform.BLUE_PARK
import org.beargineers.platform.Location
import org.beargineers.platform.PhaseBuilder
import org.beargineers.platform.PhaseDsl
import org.beargineers.platform.PhasedAutonomous
import org.beargineers.platform.Position
import org.beargineers.platform.RED_OPEN_GATE
import org.beargineers.platform.RED_OPEN_GATE_APPROACH
import org.beargineers.platform.RED_PARK
import org.beargineers.platform.abs
import org.beargineers.platform.action
import org.beargineers.platform.assumeRobotPosition
import org.beargineers.platform.closestPointInShootingZone
import org.beargineers.platform.degrees
import org.beargineers.platform.doOnce
import org.beargineers.platform.driveTo
import org.beargineers.platform.headingToGoal
import org.beargineers.platform.tilePosition
import org.beargineers.platform.wait
import kotlin.time.Duration.Companion.seconds

@PhaseDsl
private fun PhaseBuilder<DecodeRobot>.scoopSpike(spike: Spike) {
    seq("Scoop spike ${spike.name}") {
        driveTo(spike.startPose)
        driveTo(spike.endPose) // Drive in slowly, so we can carefully scoop the artifacts
     //   driveTo(spike.startPose) // Drive out carefully so we don't disturb other artifacts
    }
}

@PhaseDsl
private fun PhaseBuilder<DecodeRobot>.scoopAndShoot(spike: Spike, launchPose: Position) {
    seq("Scoop and shoot ${spike.name}") {
        scoopSpike(spike)
        shootAt(launchPose)
    }
}

private fun PhaseBuilder<DecodeRobot>.shootAt(launchPose: Position) {
    doOnce {
        enableFlywheel(true)
    }
    //  followPath(listOf(spike.startPose, spike.endPose, spike.startPose, launchPose))
    driveTo(launchPose)
    doOnce {
        launch()
    }
    waitForShootingCompletion()
}

open class DecodeAutoStrategy(alliance: Alliance, val positions: String, vararg spikes: Spike) :
    PhasedAutonomous<DecodeRobot>(alliance, {
        val (startingPoint, shootingPoint) = positions.split(",").map { it.trim() }

        autoStrategy(tilePosition(startingPoint), tilePosition(shootingPoint), *spikes)
    }) {

    override fun bearInit() {
        super.bearInit()

        telemetry.addLine("Ensure robot is in position: ${positions.takeWhile { it != ',' }}")
    }
}

@PhaseDsl
private fun PhaseBuilder<DecodeRobot>.autoStrategy(startingPoint: Position,
                                                   launchPoint: Position,
                                                   vararg spikes: Spike
) {
    assumeRobotPosition(startingPoint)
    shootInitialLoad(launchPoint)
    scoopSpike(spikes[0])
    openRamp()
    shootAt(launchPoint)
    scoopAndShoot(spikes[1], launchPoint)
    scoopAndShoot(spikes[2], launchPoint)
}

fun PhaseBuilder<DecodeRobot>.openRamp() {
    val red = opMode.alliance == Alliance.RED

    driveTo(if (red) RED_OPEN_GATE_APPROACH else BLUE_OPEN_GATE_APPROACH)
    driveTo(if (red) RED_OPEN_GATE else BLUE_OPEN_GATE, 0.8)
    wait(1.seconds)
}

@PhaseDsl
private fun PhaseBuilder<DecodeRobot>.shootInitialLoad(launchPose: Position) {
    doOnce {
        intakeMode(IntakeMode.ON)
        enableFlywheel(true)
    }
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

fun PhaseBuilder<DecodeRobot>.goToShootingZone() {
    action {
        driveToTarget(closestPointInShootingZone().withHeading(headingToGoal()), 1.0)
    }
}

fun PhaseBuilder<DecodeRobot>.park() {
    action {
        val parkCoords: Location =
            (if (opMode.alliance == Alliance.BLUE) BLUE_PARK else RED_PARK)
        val heading = currentPosition.heading
        val squareAngles = listOf(-180.degrees, -90.degrees, 0.degrees, 90.degrees, 180.degrees)
        val parkHeading = squareAngles.minBy { abs(it - heading) }
        driveToTarget(parkCoords.withHeading(parkHeading), 1.0)
    }
}

fun PhaseBuilder<DecodeRobot>.launchAndHoldPosition(position: Position) {
    action {
        launch()
        driveToTarget(position, 1.0)
        true // Keep this auto active until it is cancelled by touching gamepad controls
    }
}
