package org.beargineers

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.Alliance
import org.beargineers.platform.AutonomousPhase
import org.beargineers.platform.Phases
import org.beargineers.platform.RelativePosition
import org.beargineers.platform.action
import org.beargineers.platform.assumePosition
import org.beargineers.platform.driveRelative
import org.beargineers.platform.driveTo
import org.beargineers.platform.tilePosition
import org.beargineers.platform.wait
import org.beargineers.robot.DecodeRobot
import kotlin.time.Duration.Companion.seconds

open class TestOp(phases: Phases<DecodeRobot>) :
    DecodeAutonomous(Alliance.BLUE, phases)

@Autonomous
class Tune_HalfTileLoop : TestOp({
    wait(3.seconds)
    driveRelative(RelativePosition.forwardInch(12.0))
    driveRelative(RelativePosition.turnCCW(90.0))
    driveRelative(RelativePosition.forwardInch(12.0))
    driveRelative(RelativePosition.turnCCW(90.0))
    driveRelative(RelativePosition.forwardInch(12.0))
    driveRelative(RelativePosition.turnCCW(90.0))
    driveRelative(RelativePosition.forwardInch(12.0))
    driveRelative(RelativePosition.turnCCW(90.0))
})

@Autonomous
class Tune_OneTileLeft : TestOp({
    wait(3.seconds)
    driveRelative(RelativePosition.rightInch(-24.0))
})

@Autonomous
class Tune_Turn90CCW : TestOp({
    wait(3.seconds)
    driveRelative(RelativePosition.turnCCW(90.0))
})

@Autonomous
class Tune_C1ToC6Forward : TestOp({
    assumePosition(tilePosition("C1:180"))
    driveTo(tilePosition("C6:180"))
})

@Autonomous
class Tune_B1ToB6Left : TestOp({
    assumePosition(tilePosition("B1:90"))
    driveTo(tilePosition("B6:90"))
})

val startTestResults = mutableListOf<Double>()

class DriveRelativeUntilMoved(val forward: Double, val right: Double, val turn: Double) : AutonomousPhase<DecodeRobot> {
    var curMaxSpeed = 0.0

    override fun DecodeRobot.initPhase() {
        curMaxSpeed = 0.0
        drive.stop()
    }

    override fun DecodeRobot.loopPhase(phaseTime: ElapsedTime): Boolean {
        if (isMoving()) {
            drive.stop()
            startTestResults += curMaxSpeed
            return false
        }
        else {
            curMaxSpeed += 0.003
            drive.drive(forward * curMaxSpeed, right * curMaxSpeed, turn * curMaxSpeed)
        }
        return true
    }
}

val stopTestResults = mutableListOf<Double>()

class DriveRelativeUntilStopped(val forward: Double, val right: Double, val turn: Double) : AutonomousPhase<DecodeRobot> {
    var curMaxSpeed = 0.0

    override fun DecodeRobot.initPhase() {
        curMaxSpeed = 0.3
        drive.stop()
    }

    override fun DecodeRobot.loopPhase(phaseTime: ElapsedTime): Boolean {
        if (!isMoving() && phaseTime.seconds() > 1) {
            drive.stop()
            stopTestResults += curMaxSpeed
            return false
        }
        else {
            curMaxSpeed -= 0.003
            drive.drive(forward * curMaxSpeed, right * curMaxSpeed, turn * curMaxSpeed)
        }
        return true
    }
}

@Autonomous // Current test results: F:0.15, R:0.27, T:0.14
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

@Autonomous // Current test results: F: 0.04, R:0.11, T:0.01
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

@Autonomous
class Tune_TestWheelNames() : TestOp({
    action {
        drive.stop()
        drive.lf.power = 0.3
    }
    wait(1.seconds)
    action {
        drive.stop()
        drive.rf.power = 0.3
    }
    wait(1.seconds)
    action {
        drive.stop()
        drive.lb.power = 0.3
    }
    wait(1.seconds)
    action {
        drive.stop()
        drive.rb.power = 0.3
    }
    wait(1.seconds)
})