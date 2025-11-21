package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.cos
import kotlin.math.sin
import kotlin.time.Duration.Companion.seconds

class GotoPosePhase(val pose: Pose2D, val maxSpeed: Double) : AutonomousPhase {
    override fun Robot.initPhase() {
    }

    override fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean {
        return driveToPose(pose, maxSpeed)
    }
}

@PhaseDsl
fun PhaseBuilder.driveTo(pose: Pose2D, maxSpeed: Double = 1.0) {
    phase(GotoPosePhase(pose, maxSpeed))
}

class DriveRelative(val forward: Double, val right: Double, val turn: Double, val distanceUnit: DistanceUnit, val angleUnit: AngleUnit, val maxSpeed: Double) : AutonomousPhase {
    lateinit var targetPosition: Pose2D

    override fun Robot.initPhase() {
        val cp = currentPose.toDistanceUnit(distanceUnit).toAngleUnit(AngleUnit.RADIANS)
        val t = if (angleUnit == AngleUnit.RADIANS) turn else Math.toRadians(turn)
        targetPosition = Pose2D(
            // Forward: all wheels contribute equally
            x = cp.x + forward * cos(cp.heading) + right * sin(cp.heading),
            // Strafe: diagonal wheels oppose (LF and RB forward = strafe right)
            y = cp.y + forward * sin(cp.heading) - right * cos(cp.heading),
            // Get yaw in radians to match the angleUnit specification
            heading = cp.heading + t,
            distanceUnit = distanceUnit,
            angleUnit = AngleUnit.RADIANS)
    }

    override fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean {
        return driveToPose(targetPosition, maxSpeed)
    }
}

@PhaseDsl
fun PhaseBuilder.driveRelative(forward: Double, right: Double, turn: Double, distanceUnit: DistanceUnit, angleUnit: AngleUnit, maxSpeed: Double = 0.5) {
    phase(DriveRelative(forward, right, turn, distanceUnit, angleUnit, maxSpeed))
}

@PhaseDsl
fun PhaseBuilder.scoopSpike(spike: Spike) {
    composite("Scoop spike ${spike.name}") {
        driveTo(spike.startPose)
        driveTo(spike.endPose, maxSpeed = 0.15) // Drive in slowly, so we can carefully scoop the artifacts
        driveTo(spike.startPose) // Drive out carefully so we don't disturb other artifacts
    }
}

@PhaseDsl
fun PhaseBuilder.scoopAndShoot(spike: Spike, launchPose: Pose2D) {
    composite("Scoop and shoot ${spike.name}") {
        scoopSpike(spike)
        action {
            shooter.enableFlywheel(true)
        }
        action {
            shooter.launch()
        }
        driveTo(launchPose)
        wait(5.seconds)
    }
}

@Autonomous
class RedSouth : PhasedAutonomous(Alliance.RED, phases("Autonomous") {
    autoStrategy(
        SOUTH_RED_START_POINT,
        SOUTH_RED_LAUNCH_POINT,
        Spike.RIGHT1, Spike.RIGHT2, Spike.RIGHT3
    )
})


@Autonomous
class BlueSouth : PhasedAutonomous(Alliance.BLUE, phases {
    autoStrategy(
        SOUTH_BLUE_START_POINT,
        SOUTH_BLUE_LAUNCH_POINT,
        Spike.LEFT1, Spike.LEFT2, Spike.LEFT3
    )
})

@Autonomous
class BlueNorth : PhasedAutonomous(Alliance.BLUE, phases {
    autoStrategy(
        NORTH_BLUE_START_POINT,
        NORTH_BLUE_LAUNCH_POINT,
        Spike.LEFT3, Spike.LEFT2, Spike.LEFT1)
})

@Autonomous
class RedNorth : PhasedAutonomous(Alliance.BLUE, phases {
    autoStrategy(
        NORTH_RED_START_POINT,
        NORTH_RED_LAUNCH_POINT,
        Spike.RIGHT3, Spike.RIGHT2, Spike.RIGHT1)
})

private fun PhaseBuilder.autoStrategy(startingPoint: Pose2D,
                                      launchPoint: Pose2D,
                                      vararg spikes: Spike) {
    assumePosition(startingPoint)
    shootInitialLoad(launchPoint)
    for (spike in spikes) {
        scoopAndShoot(spike, launchPoint)
    }
}

private fun PhaseBuilder.shootInitialLoad(launchPose: Pose2D) {
    action {
        intake.enable(true)
        shooter.enableFlywheel(true)
    }
    driveTo(launchPose)
    wait(1.seconds)
    action {
        shooter.launch()
    }
    wait(5.seconds)
}

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
