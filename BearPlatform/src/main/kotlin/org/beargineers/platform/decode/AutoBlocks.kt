package org.beargineers.platform.decode

import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.Alliance
import org.beargineers.platform.AutonomousPhase
import org.beargineers.platform.PhaseBuilder
import org.beargineers.platform.PhaseDsl
import org.beargineers.platform.PhasedAutonomous
import org.beargineers.platform.Position
import org.beargineers.platform.action
import org.beargineers.platform.assumeRobotPosition
import org.beargineers.platform.driveTo
import org.beargineers.platform.tilePosition

@PhaseDsl
fun PhaseBuilder<DecodeRobot>.scoopSpike(spike: Spike) {
    seq("Scoop spike ${spike.name}") {
        driveTo(spike.startPose)
        driveTo(spike.endPose, maxSpeed = 0.15) // Drive in slowly, so we can carefully scoop the artifacts
//        driveTo(spike.startPose) // Drive out carefully so we don't disturb other artifacts
    }
}

@PhaseDsl
fun PhaseBuilder<DecodeRobot>.scoopAndShoot(spike: Spike, launchPose: Position) {
    seq("Scoop and shoot ${spike.name}") {
        scoopSpike(spike)
        action {
            enableFlywheel(true)
        }
        driveTo(launchPose)
        action {
            launch()
        }
        waitForShootingCompletion()
    }
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
    for (spike in spikes) {
        scoopAndShoot(spike, launchPoint)
    }
}

@PhaseDsl
private fun PhaseBuilder<DecodeRobot>.shootInitialLoad(launchPose: Position) {
    action {
        intakeMode(IntakeMode.ON)
        enableFlywheel(true)
    }
    driveTo(launchPose)
    action {
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