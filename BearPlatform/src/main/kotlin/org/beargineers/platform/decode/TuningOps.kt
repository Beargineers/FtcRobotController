package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.delay
import org.beargineers.platform.Alliance
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Location
import org.beargineers.platform.PIDFTCoeffs
import org.beargineers.platform.Position
import org.beargineers.platform.RobotCentricLocation
import org.beargineers.platform.RobotCentricPosition
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.cm
import org.beargineers.platform.config
import org.beargineers.platform.degrees
import org.beargineers.platform.drivePath
import org.beargineers.platform.driveRelative
import org.beargineers.platform.driveTo
import org.beargineers.platform.inch
import org.beargineers.platform.move
import org.beargineers.platform.pathTo
import org.beargineers.platform.tilePosition
import org.beargineers.platform.toFieldCentric
import kotlin.time.Duration.Companion.seconds

abstract class TestOp : RobotOpMode<DecodeRobot>() {
    override val alliance = Alliance.BLUE
}

@Autonomous(group = "Tune")
class Tune_TwoTileLoop : TestOp() {
    override suspend fun DecodeRobot.autoProgram() {
        repeat(5) {
            driveRelative(RobotCentricPosition.forward(48.inch))
            driveRelative(RobotCentricPosition.turnCCW(90.degrees))
            driveRelative(RobotCentricPosition.forward(48.inch))
            driveRelative(RobotCentricPosition.turnCCW(90.degrees))
            driveRelative(RobotCentricPosition.forward(48.inch))
            driveRelative(RobotCentricPosition.turnCCW(90.degrees))
            driveRelative(RobotCentricPosition.forward(48.inch))
            driveRelative(RobotCentricPosition.turnCCW(90.degrees))
        }
    }
}

@Autonomous(group = "Tune")
class Tune_TwoTileLoopSpline : TestOp() {
    override suspend fun DecodeRobot.autoProgram() {
        repeat(5) {
            move {
                splineTo(Location(48.inch, 0.inch), 0.degrees)
                turnTo(90.degrees)
                splineTo(Location(48.inch, 48.inch), 90.degrees)
                turnTo(180.degrees)
                splineTo(Location(0.inch, 48.inch), 180.degrees)
                turnTo(270.degrees)
                splineTo(Location(0.inch, 0.inch), 270.degrees)
                turnTo(0.degrees)
            }
        }
    }
}

@Autonomous(group = "Tune")
class Tune_OneTileLeft : TestOp() {
    override suspend fun DecodeRobot.autoProgram() {
        driveRelative(RobotCentricPosition.right(-24.inch))
    }
}

@Autonomous(group = "Tune")
class Tune_Turn90CCW : TestOp() {
    override suspend fun DecodeRobot.autoProgram() {
        driveRelative(RobotCentricPosition.turnCCW(90.degrees))
    }
}

object TuneConfig {
    val tuneDistance by config(15.cm)
    val tuneAngle by config(15.degrees)
}



@Autonomous(group = "Tune")
class Tune_driveK : TestOp() {
    override suspend fun DecodeRobot.autoProgram() {
        doTuning(tilePosition("B1:90"), {
            RobotCentricLocation(TuneConfig.tuneDistance, 0.cm).toFieldCentric(it).withHeading(it.heading)
        },{
            RobotCentricLocation(-TuneConfig.tuneDistance, 0.cm).toFieldCentric(it).withHeading(it.heading)
        }){it.drive_K}
    }
}

@Autonomous(group = "Tune")
class Tune_translationK : TestOp() {
    override suspend fun DecodeRobot.autoProgram() {
        doTuning(tilePosition("B2:180"), {
            RobotCentricLocation(0.cm, -TuneConfig.tuneDistance).toFieldCentric(it).withHeading(it.heading)
        },{
            RobotCentricLocation(0.cm, TuneConfig.tuneDistance).toFieldCentric(it).withHeading(it.heading)
        }){it.translational_K}
    }
}

@Autonomous(group = "Tune")
class Tune_headingK: TestOp() {
    override suspend fun DecodeRobot.autoProgram() {
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
    override suspend fun DecodeRobot.autoProgram() {
        val startingPoint = AutoPositions.NORTH_START.mirrorForAlliance(Alliance.BLUE)
        val launchPoint = AutoPositions.NORTH_SHOOTING.mirrorForAlliance(Alliance.BLUE)
        repeat(1) {
            val timer = ElapsedTime()

            assumePosition(startingPoint)

            followPathAndShoot(pathTo(launchPoint, locations.INITIAL_SHOT_SPEED))

            val secondScoop = scoopSpikePath(2)
            drivePath(secondScoop + openRampPath())

            delay(1.seconds)
            followPathAndShoot(pathTo(launchPoint))
            // Far shooting zone
            scoopAndShoot(3, launchPoint)
            scoopAndShoot(1, launchPoint)

            println("TUNING_TIMER: ${timer.milliseconds().toInt()} ms drive: ${(robot as BaseRobot).drive_K} translation: ${(robot as BaseRobot).translational_K} heading: ${(robot as BaseRobot).heading_K} drive2: ${(robot as BaseRobot).drive_K2}")

            driveTo(startingPoint)
        }
    }
}

private suspend fun DecodeRobot.doTuning(
    startingPos: Position,
    shift:(pos: Position)->Position,
    shiftBack:(pos: Position)->Position,
    coeff: (robot: BaseRobot)-> PIDFTCoeffs
) {
    assumePosition(startingPos)

    repeat(1000) {

        val forth = pathTo(shift(currentPosition))

        val timer = System.currentTimeMillis()
        drivePath(forth)

        val error = forth.first().target.distanceTo(currentPosition)
        println("TUNETIME: ${System.currentTimeMillis() - timer} ERROR: ${error.cm()}cm PID: ${coeff((this as BaseRobot))}}")

        delay(0.5.seconds)

        val back = pathTo(shiftBack(currentPosition), speed = locations.INITIAL_SHOT_SPEED)
        drivePath(back)

        delay(0.5.seconds)

        //s_followPath(pathTo(startingPos))
        //delay(0.5.seconds)

        /*
            * heading_K=0.019, 0.0, 0.0015, 0.3
drive_K=0.03, 0.00001, 0.003, 0
translational_K=0.025, 0.002, 0.0035, 0.3
            * */
    }
}

@Autonomous(group = "Tune")
class Tune_C1ToC6Forward : TestOp() {
    override suspend fun DecodeRobot.autoProgram() {
        assumePosition(tilePosition("C1:180"))
        driveTo(tilePosition("C6:180"))
    }
}

@Autonomous(group = "Tune")
class Tune_B1ToB6Left : TestOp() {
    override suspend fun DecodeRobot.autoProgram() {
        assumePosition(tilePosition("B1:90"))
        driveTo(tilePosition("B6:90"))
    }
}
