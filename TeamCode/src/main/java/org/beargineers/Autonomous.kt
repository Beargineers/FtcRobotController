package org.beargineers

import org.beargineers.platform.Alliance
import org.beargineers.platform.PhaseBuilder
import org.beargineers.platform.PhaseDsl
import org.beargineers.platform.Position
import org.beargineers.platform.action
import org.beargineers.platform.assumePosition
import org.beargineers.platform.driveTo
import org.beargineers.platform.tilePosition
import org.beargineers.platform.wait
import kotlin.time.Duration.Companion.seconds

@PhaseDsl
fun PhaseBuilder<DecodeRobot>.scoopSpike(spike: Spike) {
    composite("Scoop spike ${spike.name}") {
        driveTo(spike.startPose)
        driveTo(spike.endPose, maxSpeed = 0.15) // Drive in slowly, so we can carefully scoop the artifacts
        driveTo(spike.startPose) // Drive out carefully so we don't disturb other artifacts
    }
}

@PhaseDsl
fun PhaseBuilder<DecodeRobot>.scoopAndShoot(spike: Spike, launchPose: Position) {
    composite("Scoop and shoot ${spike.name}") {
        scoopSpike(spike)
        action {
            shooter.enableFlywheel(true)
        }
        driveTo(launchPose)
        action {
            shooter.launch()
        }
        wait(ShooterConfig.SHOOTING_TIME_SECONDS.seconds)
    }
}

open class DecodeAutoStrategy(alliance: Alliance, val positions: String, vararg spikes: Spike) :
    DecodeAutonomous(alliance, {
        val (startingPoint, shootingPoint) = positions.split(",").map { it.trim() }

        autoStrategy(tilePosition(startingPoint), tilePosition(shootingPoint), *spikes)
    }) {

    override fun init() {
        super.init()

        telemetry.addLine("Ensure robot is in position: ${positions.takeWhile { it != ',' }}")
    }
}

@PhaseDsl
private fun PhaseBuilder<DecodeRobot>.autoStrategy(startingPoint: Position,
                                                   launchPoint: Position,
                                                   vararg spikes: Spike) {
    assumePosition(startingPoint)
    shootInitialLoad(launchPoint)
    scoopAndShoot(spikes[0], launchPoint)
    scoopSpike(spikes[1])

}

@PhaseDsl
private fun PhaseBuilder<DecodeRobot>.shootInitialLoad(launchPose: Position) {
    action {
        intake.enable(IntakeMode.ON)
        shooter.enableFlywheel(true)
    }
    driveTo(launchPose)
    wait(1.seconds)
    waitForDistance()
    action {
        shooter.launch()
        shooter.defaultGoalDistance = savedGoalDistanceCM
    }
    wait(ShooterConfig.SHOOTING_TIME_SECONDS.seconds)
}
