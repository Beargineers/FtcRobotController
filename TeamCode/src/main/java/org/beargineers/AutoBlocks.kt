package org.beargineers

import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.Alliance
import org.beargineers.platform.AutonomousPhase
import org.beargineers.platform.PhaseBuilder
import org.beargineers.platform.PhaseDsl
import org.beargineers.platform.Position
import org.beargineers.platform.action
import org.beargineers.platform.assumePosition
import org.beargineers.platform.driveTo
import org.beargineers.platform.tilePosition
import org.beargineers.platform.wait
import org.beargineers.robot.DecodeRobot
import org.beargineers.robot.IntakeMode
import org.beargineers.robot.ShooterConfig
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds

@PhaseDsl
fun PhaseBuilder<DecodeRobot>.scoopSpike(spike: Spike) {
    seq("Scoop spike ${spike.name}") {
        driveTo(spike.startPose)
        driveTo(spike.endPose, maxSpeed = 0.15) // Drive in slowly, so we can carefully scoop the artifacts
        driveTo(spike.startPose) // Drive out carefully so we don't disturb other artifacts
    }
}

@PhaseDsl
fun PhaseBuilder<DecodeRobot>.scoopAndShoot(spike: Spike, launchPose: Position) {
    seq("Scoop and shoot ${spike.name}") {
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

    override fun bearInit() {
        super.bearInit()

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

class WaitForDistance(val timeoutSec: Int) : AutonomousPhase<DecodeRobot> {
    override fun DecodeRobot.initPhase() {
        drive.stop()
    }

    override fun DecodeRobot.loopPhase(phaseTime: ElapsedTime): Boolean {
        goalDistanceCM?.let {
            savedGoalDistanceCM = it
            return false
        }
        return phaseTime.seconds() < timeoutSec
    }
}

@PhaseDsl
fun PhaseBuilder<DecodeRobot>.waitForDistance(timeoutSec: Duration = 3.seconds) {
    phase(WaitForDistance(timeoutSec.inWholeSeconds.toInt()))
}