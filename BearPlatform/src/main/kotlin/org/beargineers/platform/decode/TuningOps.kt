package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.Alliance
import org.beargineers.platform.AutonomousPhase
import org.beargineers.platform.PhaseBuilder
import org.beargineers.platform.PhasedAutonomous
import org.beargineers.platform.RelativePosition
import org.beargineers.platform.Robot
import org.beargineers.platform.assumeRobotPosition
import org.beargineers.platform.degrees
import org.beargineers.platform.drive
import org.beargineers.platform.driveRelative
import org.beargineers.platform.inch
import org.beargineers.platform.pathTo
import org.beargineers.platform.tilePosition
import org.beargineers.platform.wait
import kotlin.time.Duration.Companion.seconds

abstract class TestOp() :
    PhasedAutonomous<DecodeRobot>(Alliance.BLUE)

@Autonomous(group = "Tune")
class Tune_HalfTileLoop : TestOp() {
    override fun PhaseBuilder<DecodeRobot>.phases() {
        wait(3.seconds)
        driveRelative(RelativePosition.forward(12.inch))
        driveRelative(RelativePosition.turnCCW(90.degrees))
        driveRelative(RelativePosition.forward(12.inch))
        driveRelative(RelativePosition.turnCCW(90.degrees))
        driveRelative(RelativePosition.forward(12.inch))
        driveRelative(RelativePosition.turnCCW(90.degrees))
        driveRelative(RelativePosition.forward(12.inch))
        driveRelative(RelativePosition.turnCCW(90.degrees))
    }
}

@Autonomous(group = "Tune")
class Tune_OneTileLeft : TestOp() {
    override fun PhaseBuilder<DecodeRobot>.phases() {
        wait(3.seconds)
        driveRelative(RelativePosition.right(-24.inch))
    }
}

@Autonomous(group = "Tune")
class Tune_Turn90CCW : TestOp() {
    override fun PhaseBuilder<DecodeRobot>.phases() {
        wait(3.seconds)
        driveRelative(RelativePosition.turnCCW(90.degrees))
    }
}

@Autonomous(group = "Tune")
class Tune_C1ToC6Forward : TestOp() {
    override fun PhaseBuilder<DecodeRobot>.phases() {
        assumeRobotPosition(tilePosition("C1:180"))
        drive(pathTo(tilePosition("C6:180")))
    }
}

@Autonomous(group = "Tune")
class Tune_B1ToB6Left : TestOp() {
    override fun PhaseBuilder<DecodeRobot>.phases() {
        assumeRobotPosition(tilePosition("B1:90"))
        drive(pathTo(tilePosition("B6:90")))
    }
}

val startTestResults = mutableListOf<Double>()

class DriveRelativeUntilMoved(val forward: Double, val right: Double, val turn: Double) : AutonomousPhase<Robot> {
    var curMaxSpeed = 0.0

    override fun Robot.initPhase() {
        curMaxSpeed = 0.0
        stopDriving()
    }

    override fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean {
        if (isMoving()) {
            stopDriving()
            startTestResults += curMaxSpeed
            return false
        }
        else {
            curMaxSpeed += 0.003
            drive(forward * curMaxSpeed, right * curMaxSpeed, turn * curMaxSpeed)
        }
        return true
    }
}

val stopTestResults = mutableListOf<Double>()

class DriveRelativeUntilStopped(val forward: Double, val right: Double, val turn: Double) : AutonomousPhase<Robot> {
    var curMaxSpeed = 0.0

    override fun Robot.initPhase() {
        curMaxSpeed = 0.3
        stopDriving()
    }

    override fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean {
        if (!isMoving() && phaseTime.seconds() > 1) {
            stopDriving()
            stopTestResults += curMaxSpeed
            return false
        }
        else {
            curMaxSpeed -= 0.003
            drive(forward * curMaxSpeed, right * curMaxSpeed, turn * curMaxSpeed)
        }
        return true
    }
}

@Autonomous(group = "Tune")
// Current test results: F:0.15, R:0.27, T:0.14
class Tune_StartingPowers() : TestOp() {
    override fun PhaseBuilder<DecodeRobot>.phases() {
        val forward = DriveRelativeUntilMoved(1.0, 0.0, 0.0)
        val right = DriveRelativeUntilMoved(0.0, 1.0, 0.0)
        val turn = DriveRelativeUntilMoved(0.0, 0.0, 1.0)

        phase(forward)
        phase(right)
        phase(turn)
        wait(30.seconds)
    }

    override fun bearLoop() {
        super.bearLoop()

        if (startTestResults.size > 0) telemetry.addData("Moved F at speed", "%.2f", startTestResults[0])
        if (startTestResults.size > 1) telemetry.addData("Moved R at speed", "%.2f", startTestResults[1])
        if (startTestResults.size > 2) telemetry.addData("Moved T at speed", "%.2f", startTestResults[2])
    }
}

@Autonomous(group = "Tune")
// Current test results: F: 0.04, R:0.11, T:0.01
class Tune_StoppingPowers() : TestOp() {
    override fun PhaseBuilder<DecodeRobot>.phases() {
        val forward = DriveRelativeUntilStopped(1.0, 0.0, 0.0)
        val right = DriveRelativeUntilStopped(0.0, 1.0, 0.0)
        val turn = DriveRelativeUntilStopped(0.0, 0.0, 1.0)

        phase(forward)
        phase(right)
        phase(turn)
        wait(30.seconds)
    }

    override fun bearLoop() {
        super.bearLoop()

        if (stopTestResults.size > 0) telemetry.addData("Stopped F at speed", "%.2f", stopTestResults[0])
        if (stopTestResults.size > 1) telemetry.addData("Stopped R at speed", "%.2f", stopTestResults[1])
        if (stopTestResults.size > 2) telemetry.addData("Stopped T at speed", "%.2f", stopTestResults[2])
    }
}
