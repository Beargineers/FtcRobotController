package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.time.Duration.Companion.seconds

open class TestOp(rootPhase: CompositePhase) : PhasedAutonomous(Alliance.BLUE, rootPhase)

@Autonomous
class Tune_HalfTileLoop : TestOp(phases {
    wait(3.seconds)
    driveRelative(12.0, 0.0, 0.0, DistanceUnit.INCH, AngleUnit.RADIANS)
    driveRelative(0.0, 0.0, 90.0, DistanceUnit.INCH, AngleUnit.DEGREES)
    driveRelative(12.0, 0.0, 0.0, DistanceUnit.INCH, AngleUnit.RADIANS)
    driveRelative(0.0, 0.0, 90.0, DistanceUnit.INCH, AngleUnit.DEGREES)
    driveRelative(12.0, 0.0, 0.0, DistanceUnit.INCH, AngleUnit.RADIANS)
    driveRelative(0.0, 0.0, 90.0, DistanceUnit.INCH, AngleUnit.DEGREES)
    driveRelative(12.0, 0.0, 0.0, DistanceUnit.INCH, AngleUnit.RADIANS)
    driveRelative(0.0, 0.0, 90.0, DistanceUnit.INCH, AngleUnit.DEGREES)
})

@Autonomous
class Tune_OneTileLeft : TestOp(phases {
    wait(3.seconds)
    driveRelative(0.0, -24.0, 0.0, DistanceUnit.INCH, AngleUnit.RADIANS)
})

@Autonomous
class Tune_Turn90CCW : TestOp(phases {
    wait(3.seconds)
    driveRelative(0.0, 0.0, 90.0, DistanceUnit.INCH, AngleUnit.DEGREES)
})

@Autonomous
class Tune_C1ToC6Forward : TestOp(phases {
    assumePosition(tilePosition("C1").withHeading(180.0, AngleUnit.DEGREES))
    driveTo(tilePosition("C6").withHeading(180.0, AngleUnit.DEGREES))
})

@Autonomous
class Tune_B1ToB6Left : TestOp(phases {
    assumePosition(tilePosition("B1").withHeading(90.0, AngleUnit.DEGREES))
    driveTo(tilePosition("B6").withHeading(90.0, AngleUnit.DEGREES))
})
