package org.beargineers

import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.AutonomousPhase
import org.beargineers.platform.PhaseBuilder
import org.beargineers.platform.PhaseDsl
import kotlin.time.Duration
import kotlin.time.Duration.Companion.seconds

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
