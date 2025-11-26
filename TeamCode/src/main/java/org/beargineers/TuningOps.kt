package org.beargineers

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.beargineers.platform.Alliance
import org.beargineers.platform.Phases
import org.beargineers.platform.RobotMovement
import org.beargineers.platform.assumePosition
import org.beargineers.platform.driveRelative
import org.beargineers.platform.driveTo
import org.beargineers.platform.tilePosition
import org.beargineers.platform.wait
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