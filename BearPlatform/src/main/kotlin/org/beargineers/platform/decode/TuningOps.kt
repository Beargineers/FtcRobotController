package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.Alliance
import org.beargineers.platform.AutonomousPhase
import org.beargineers.platform.CurveToPosePhase
import org.beargineers.platform.PhasedAutonomous
import org.beargineers.platform.Phases
import org.beargineers.platform.RelativePosition
import org.beargineers.platform.Robot
import org.beargineers.platform.assumeRobotPosition
import org.beargineers.platform.cm
import org.beargineers.platform.degrees
import org.beargineers.platform.driveRelative
import org.beargineers.platform.driveTo
import org.beargineers.platform.inch
import org.beargineers.platform.tilePosition
import org.beargineers.platform.wait
import kotlin.time.Duration.Companion.seconds

open class TestOp(phases: Phases<DecodeRobot>) :
    PhasedAutonomous<DecodeRobot>(Alliance.BLUE, phases)

@Autonomous(group = "Tune")
class Tune_HalfTileLoop : TestOp({
    wait(3.seconds)
    driveRelative(RelativePosition.forward(12.inch))
    driveRelative(RelativePosition.turnCCW(90.degrees))
    driveRelative(RelativePosition.forward(12.inch))
    driveRelative(RelativePosition.turnCCW(90.degrees))
    driveRelative(RelativePosition.forward(12.inch))
    driveRelative(RelativePosition.turnCCW(90.degrees))
    driveRelative(RelativePosition.forward(12.inch))
    driveRelative(RelativePosition.turnCCW(90.degrees))
})

@Autonomous(group = "Tune")
class Tune_OneTileLeft : TestOp({
    wait(3.seconds)
    driveRelative(RelativePosition.right(-24.inch))
})

@Autonomous(group = "Tune")
class Tune_Turn90CCW : TestOp({
    wait(3.seconds)
    driveRelative(RelativePosition.turnCCW(90.degrees))
})

@Autonomous(group = "Tune")
class Tune_C1ToC6Forward : TestOp({
    assumeRobotPosition(tilePosition("C1:180"))
    driveTo(tilePosition("C6:180"))
})

@Autonomous(group = "Tune")
class Tune_B1ToB6Left : TestOp({
    assumeRobotPosition(tilePosition("B1:90"))
    driveTo(tilePosition("B6:90"))
})

@Autonomous(group = "Tune")
class CurveTest : PhasedAutonomous<DecodeRobot>(Alliance.BLUE, {
    assumeRobotPosition(tilePosition("B3:-90"))
    phase(CurveToPosePhase(tilePosition("B5:90"), 70.cm, false))
})

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
class Tune_StartingPowers() : TestOp({
    val forward = DriveRelativeUntilMoved(1.0, 0.0, 0.0)
    val right = DriveRelativeUntilMoved(0.0, 1.0, 0.0)
    val turn = DriveRelativeUntilMoved(0.0, 0.0, 1.0)

    phase(forward)
    phase(right)
    phase(turn)
    wait(30.seconds)
}) {
    override fun bearLoop() {
        super.bearLoop()

        if (startTestResults.size > 0) telemetry.addData("Moved F at speed", "%.2f", startTestResults[0])
        if (startTestResults.size > 1) telemetry.addData("Moved R at speed", "%.2f", startTestResults[1])
        if (startTestResults.size > 2) telemetry.addData("Moved T at speed", "%.2f", startTestResults[2])
    }
}

@Autonomous(group = "Tune")
// Current test results: F: 0.04, R:0.11, T:0.01
class Tune_StoppingPowers() : TestOp({
    val forward = DriveRelativeUntilStopped(1.0, 0.0, 0.0)
    val right = DriveRelativeUntilStopped(0.0, 1.0, 0.0)
    val turn = DriveRelativeUntilStopped(0.0, 0.0, 1.0)

    phase(forward)
    phase(right)
    phase(turn)
    wait(30.seconds)
}) {
    override fun bearLoop() {
        super.bearLoop()

        if (stopTestResults.size > 0) telemetry.addData("Stopped F at speed", "%.2f", stopTestResults[0])
        if (stopTestResults.size > 1) telemetry.addData("Stopped R at speed", "%.2f", stopTestResults[1])
        if (stopTestResults.size > 2) telemetry.addData("Stopped T at speed", "%.2f", stopTestResults[2])
    }
}
