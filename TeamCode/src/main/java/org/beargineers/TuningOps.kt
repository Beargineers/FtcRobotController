package org.beargineers

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.Alliance
import org.beargineers.platform.AutonomousPhase
import org.beargineers.platform.Phases
import org.beargineers.platform.RobotMovement
import org.beargineers.platform.assumePosition
import org.beargineers.platform.driveRelative
import org.beargineers.platform.driveTo
import org.beargineers.platform.tilePosition
import org.beargineers.platform.wait
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.abs
import kotlin.time.Duration.Companion.seconds

open class TestOp(phases: Phases<DecodeRobot>) :
    DecodeAutonomous(Alliance.BLUE, phases)

@Autonomous
class Tune_HalfTileLoop : TestOp({
    wait(3.seconds)
    driveRelative(RobotMovement.forwardInch(12.0))
    driveRelative(RobotMovement.turnCCW(90.0))
    driveRelative(RobotMovement.forwardInch(12.0))
    driveRelative(RobotMovement.turnCCW(90.0))
    driveRelative(RobotMovement.forwardInch(12.0))
    driveRelative(RobotMovement.turnCCW(90.0))
    driveRelative(RobotMovement.forwardInch(12.0))
    driveRelative(RobotMovement.turnCCW(90.0))
})

@Autonomous
class Tune_OneTileLeft : TestOp({
    wait(3.seconds)
    driveRelative(RobotMovement.rightInch(-24.0))
})

@Autonomous
class Tune_Turn90CCW : TestOp({
    wait(3.seconds)
    driveRelative(RobotMovement.turnCCW(90.0))
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
    var move: RobotMovement = RobotMovement(0.0, 0.0, 0.0, DistanceUnit.CM, AngleUnit.DEGREES)

    override fun DecodeRobot.initPhase() {
        curMaxSpeed = 0.0
        drive.stop()
    }

    fun shifted(): Boolean {
        val cm = move.toUnits(DistanceUnit.CM, AngleUnit.DEGREES)
        return abs(cm.forward) > 1.0 || abs(cm.right) > 1.0 || abs(cm.turn) > 2.0
    }

    override fun DecodeRobot.loopPhase(phaseTime: ElapsedTime): Boolean {
        move = currentMove
        if (shifted()) {
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
    var move: RobotMovement = RobotMovement(0.0, 0.0, 0.0, DistanceUnit.CM, AngleUnit.DEGREES)

    override fun DecodeRobot.initPhase() {
        curMaxSpeed = 0.3
        drive.stop()
    }

    fun shifted(): Boolean {
        val cm = move.toUnits(DistanceUnit.CM, AngleUnit.DEGREES)
        return abs(cm.forward) > 0.0 || abs(cm.right) > 0.0 || abs(cm.turn) > 0.0
    }

    override fun DecodeRobot.loopPhase(phaseTime: ElapsedTime): Boolean {
        move = currentMove
        if (!shifted() && phaseTime.seconds() > 1) {
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