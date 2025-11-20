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
        //TODO switch intake on
        driveTo(spike.endPose, maxSpeed = 0.2) // Drive in slowly, so we can carefully scoop the artifacts
        //TODO switch intake off
        driveTo(spike.startPose, maxSpeed = 0.5) // Drive out carefully, so we don't disturb nearby artifacts
    }
}

@PhaseDsl
fun PhaseBuilder.scoopAndShoot(spike: Spike, launchPose: Pose2D) {
    composite("Scoop and shoot ${spike.name}") {
        scoopSpike(spike)
        driveTo(launchPose)
        //TODO shoot
    }
}

@Autonomous(name = "Red Alliance South")
class RedSouthAutonomous : PhasedAutonomous(phases("Autonomous") {
    scoopAndShoot(Spike.RIGHT1, SOUTH_RED_LAUNCH_POINT)
    scoopAndShoot(Spike.RIGHT2, SOUTH_RED_LAUNCH_POINT)
    scoopAndShoot(Spike.RIGHT3, SOUTH_RED_LAUNCH_POINT)
})

@Autonomous
class HalfTileLoop : PhasedAutonomous(phases {
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
class OneTileLeft : PhasedAutonomous(phases {
    wait(3.seconds)
    driveRelative(0.0, -24.0, 0.0, DistanceUnit.INCH, AngleUnit.RADIANS)
})

@Autonomous
class Turn90CCW : PhasedAutonomous(phases {
    wait(3.seconds)
    driveRelative(0.0, 0.0, 90.0, DistanceUnit.INCH, AngleUnit.DEGREES)
})