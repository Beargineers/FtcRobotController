package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.Alliance
import org.beargineers.platform.AutonomousPhase
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Location
import org.beargineers.platform.PIDFTCoeffs
import org.beargineers.platform.PhaseBuilder
import org.beargineers.platform.PhasedAutonomous
import org.beargineers.platform.Position
import org.beargineers.platform.RelativePosition
import org.beargineers.platform.Robot
import org.beargineers.platform.Waypoint
import org.beargineers.platform.assumeRobotPosition
import org.beargineers.platform.cm
import org.beargineers.platform.config
import org.beargineers.platform.degrees
import org.beargineers.platform.doOnce
import org.beargineers.platform.drive
import org.beargineers.platform.driveRelative
import org.beargineers.platform.inch
import org.beargineers.platform.pathTo
import org.beargineers.platform.tilePosition
import org.beargineers.platform.toAbsolute
import org.beargineers.platform.wait
import kotlin.time.Duration.Companion.seconds

abstract class TestOp() :
    PhasedAutonomous<DecodeRobot>() {
    override val alliance = Alliance.BLUE
}

@Autonomous(group = "Tune")
class Tune_TwoTileLoop : TestOp() {
    override fun PhaseBuilder<DecodeRobot>.phases() {
        repeat(5) {
            driveRelative(RelativePosition.forward(48.inch))
            driveRelative(RelativePosition.turnCCW(90.degrees))
            driveRelative(RelativePosition.forward(48.inch))
            driveRelative(RelativePosition.turnCCW(90.degrees))
            driveRelative(RelativePosition.forward(48.inch))
            driveRelative(RelativePosition.turnCCW(90.degrees))
            driveRelative(RelativePosition.forward(48.inch))
            driveRelative(RelativePosition.turnCCW(90.degrees))
        }
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

object TuneConfig {
    val tuneDistance by config(15.cm)
    val tuneAngle by config(15.degrees)
}



@Autonomous(group = "Tune")
class Tune_driveK: TestOp() {
    override fun PhaseBuilder<DecodeRobot>.phases(){
        doTuning(tilePosition("B1:90"), {
            Location(0.cm, TuneConfig.tuneDistance).toAbsolute(it).withHeading(it.heading)
        },{
            Location(0.cm, -TuneConfig.tuneDistance).toAbsolute(it).withHeading(it.heading)
        }){it.drive_K}
    }
}

@Autonomous(group = "Tune")
class Tune_translationK: TestOp() {
    override fun PhaseBuilder<DecodeRobot>.phases(){
        doTuning(tilePosition("B2:180"), {
            Location(-TuneConfig.tuneDistance, 0.cm).toAbsolute(it).withHeading(it.heading)
        },{
            Location(TuneConfig.tuneDistance, 0.cm).toAbsolute(it).withHeading(it.heading)
        }){it.translational_K}
    }
}

@Autonomous(group = "Tune")
class Tune_headingK: TestOp() {
    override fun PhaseBuilder<DecodeRobot>.phases(){
        doTuning(tilePosition("B2:180"), {
            it.location().withHeading(it.heading.plus(TuneConfig.tuneAngle))
        },{
            it.location().withHeading(it.heading.minus(TuneConfig.tuneAngle))
        }){it.heading_K}
    }
}
/**
 *
 * heading_K=0.02, 0.04, 0.001
 * drive_K=0.02, 0.002, 0.003
 * drive_K2=0.05, 0.002, 0.008
 * translational_K=0.0253, 0.008, 0.004
 */

@Autonomous(group = "Tune")
class Tune_automationTimingCheck: TestOp() {
    override fun PhaseBuilder<DecodeRobot>.phases(){
        val startingPoint = AutoPositions.NORTH_START.mirrorForAlliance(Alliance.BLUE)
        val launchPoint = AutoPositions.NORTH_SHOOTING.mirrorForAlliance(Alliance.BLUE)

        repeat(1) {
            val timers = ArrayList<Long>()

            doOnce { timers.add(System.currentTimeMillis()) }

            assumeRobotPosition(startingPoint)

            followPathAndShoot(pathTo(launchPoint, robot.locations.INITIAL_SHOT_SPEED))
            val secondScoop = robot.scoopSpikePath(2)

            drive(secondScoop + robot.openRampPath())

            wait(1.seconds)
            followPathAndShoot(pathTo(launchPoint))
            // Far shooting zone
            scoopAndShoot(3, launchPoint)
            scoopAndShoot(1, launchPoint)

            doOnce {
                println("TUNING_TIMER: ${System.currentTimeMillis() - timers.first()} ms drive: ${(robot as BaseRobot).drive_K} translation: ${(robot as BaseRobot).translational_K} heading: ${(robot as BaseRobot).heading_K} drive2: ${(robot as BaseRobot).drive_K2}" )
            }

            drive(pathTo(startingPoint))
        }

    }
}

private fun PhaseBuilder<DecodeRobot>.doTuning(
    startingPos: Position,
    shift:(pos: Position)->Position,
    shiftBack:(pos: Position)->Position,
    coeff: (robot: BaseRobot)-> PIDFTCoeffs
) {
    assumeRobotPosition(startingPos)

    repeat(1000) {

        val forth = mutableListOf<Waypoint>()
        val back = mutableListOf<Waypoint>()

        doOnce {
            forth.clear()
            forth.addAll(pathTo(shift(robot.currentPosition)))
        }
        val timer = ArrayList<Long>()
        doOnce { timer.add(System.currentTimeMillis()) }
        drive(forth)
        doOnce {
            val error = forth.first().target.distanceTo(robot.currentPosition)
            println("TUNETIME: ${System.currentTimeMillis() - timer.first()} ERROR: ${error.cm()}cm PID: ${coeff((robot as BaseRobot))}}")

        }

        wait(0.5.seconds)

        doOnce {
            back.clear()
            back.addAll(pathTo(shiftBack(robot.currentPosition), speed = robot.locations.INITIAL_SHOT_SPEED))
        }

        drive(back)

        wait(0.5.seconds)

        //drive(pathTo(startingPos))
        //wait(0.5.seconds)

        /*
            * heading_K=0.019, 0.0, 0.0015, 0.3
drive_K=0.03, 0.00001, 0.003, 0
translational_K=0.025, 0.002, 0.0035, 0.3
            * */
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
