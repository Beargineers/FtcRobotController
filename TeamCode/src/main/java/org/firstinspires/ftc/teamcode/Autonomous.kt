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
        driveTo(spike.endPose, maxSpeed = 0.2) // Drive in slowly, so we can carefully scoop the artifacts
        driveTo(spike.startPose) // Drive out carefully so we don't disturb other artifacts
    }
}

@PhaseDsl
fun PhaseBuilder.scoopAndShoot(spike: Spike, launchPose: Pose2D) {
    composite("Scoop and shoot ${spike.name}") {
        action {
            intake.enable(true)
        }
        scoopSpike(spike)
        action {
            shooter.enableFlywheel(true)
        }
        driveTo(launchPose)
        action {
            shooter.launch()
        }
        wait(4.seconds)
        action {
            shooter.enableFlywheel(false)
            intake.enable(false)
        }
    }
}

@Autonomous
class RedSouth : PhasedAutonomous(phases("Autonomous") {
    assumePosition(tilePosition("D1").withHeading(180.0, AngleUnit.DEGREES))
    scoopAndShoot(Spike.RIGHT1, SOUTH_RED_LAUNCH_POINT)
    scoopAndShoot(Spike.RIGHT2, SOUTH_RED_LAUNCH_POINT)
    scoopAndShoot(Spike.RIGHT3, SOUTH_RED_LAUNCH_POINT)
})

@Autonomous
class BlueSouth : PhasedAutonomous(phases("Autonomous") {
    assumePosition(tilePosition("C1").withHeading(180.0, AngleUnit.DEGREES))
    scoopAndShoot(Spike.LEFT1, SOUTH_BLUE_LAUNCH_POINT)
    scoopAndShoot(Spike.LEFT2, SOUTH_BLUE_LAUNCH_POINT)
    scoopAndShoot(Spike.LEFT3, SOUTH_BLUE_LAUNCH_POINT)
})

@Autonomous
class BlueNorth : PhasedAutonomous(phases("Autonomous") {
    assumePosition(NORTH_BLUE_LAUNCH_POINT)
    scoopAndShoot(Spike.LEFT3, NORTH_BLUE_LAUNCH_POINT)
    scoopAndShoot(Spike.LEFT2, NORTH_BLUE_LAUNCH_POINT)
    scoopAndShoot(Spike.LEFT1, NORTH_BLUE_LAUNCH_POINT)
})

@Autonomous
class Tune_HalfTileLoop : PhasedAutonomous(phases {
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
class Tune_OneTileLeft : PhasedAutonomous(phases {
    wait(3.seconds)
    driveRelative(0.0, -24.0, 0.0, DistanceUnit.INCH, AngleUnit.RADIANS)
})

@Autonomous
class Tune_Turn90CCW : PhasedAutonomous(phases {
    wait(3.seconds)
    driveRelative(0.0, 0.0, 90.0, DistanceUnit.INCH, AngleUnit.DEGREES)
})

@Autonomous
class Tune_C1ToC6Forward : PhasedAutonomous(phases {
    assumePosition(tilePosition("C1").withHeading(180.0, AngleUnit.DEGREES))
    driveTo(tilePosition("C6").withHeading(180.0, AngleUnit.DEGREES))
})

@Autonomous
class Tune_B1ToB6Left : PhasedAutonomous(phases {
    assumePosition(tilePosition("B1").withHeading(90.0, AngleUnit.DEGREES))
    driveTo(tilePosition("B6").withHeading(90.0, AngleUnit.DEGREES))
})
