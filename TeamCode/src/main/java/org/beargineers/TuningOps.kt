package org.beargineers

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.beargineers.platform.Alliance
import org.beargineers.platform.PhaseBuilder
import org.beargineers.platform.RobotMovement
import org.beargineers.platform.assumePosition
import org.beargineers.platform.driveRelative
import org.beargineers.platform.driveTo
import org.beargineers.platform.tilePosition
import org.beargineers.platform.wait
import kotlin.time.Duration.Companion.seconds

abstract class TestOp() : DecodeAutonomous(Alliance.BLUE)

@Autonomous
class Tune_HalfTileLoop : TestOp() {
    override fun PhaseBuilder<DecodeRobot>.createPhases() {
        wait(3.seconds)
        driveRelative(RobotMovement.forwardInch(12.0))
        driveRelative(RobotMovement.turnCCW(90.0))
        driveRelative(RobotMovement.forwardInch(12.0))
        driveRelative(RobotMovement.turnCCW(90.0))
        driveRelative(RobotMovement.forwardInch(12.0))
        driveRelative(RobotMovement.turnCCW(90.0))
        driveRelative(RobotMovement.forwardInch(12.0))
        driveRelative(RobotMovement.turnCCW(90.0))
    }
}

@Autonomous
class Tune_OneTileLeft : TestOp() {
    override fun PhaseBuilder<DecodeRobot>.createPhases() {
        wait(3.seconds)
        driveRelative(RobotMovement.rightInch(-24.0))
    }
}

@Autonomous
class Tune_Turn90CCW : TestOp() {
    override fun PhaseBuilder<DecodeRobot>.createPhases() {
        wait(3.seconds)
        driveRelative(RobotMovement.turnCCW(90.0))
    }
}

@Autonomous
class Tune_C1ToC6Forward : TestOp() {
    override fun PhaseBuilder<DecodeRobot>.createPhases() {
        assumePosition(tilePosition("C1:180"))
        driveTo(tilePosition("C6:180"))
    }
}

@Autonomous
class Tune_B1ToB6Left : TestOp() {
    override fun PhaseBuilder<DecodeRobot>.createPhases() {
        assumePosition(tilePosition("B1:90"))
        driveTo(tilePosition("B6:90"))
    }
}