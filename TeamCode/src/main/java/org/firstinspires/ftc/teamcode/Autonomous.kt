package org.firstinspires.ftc.teamcode

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
        driveTo(launchPose)
        action {
            shooter.launch()
        }
        wait(ShooterConfig.SHOOTING_TIME_SECONDS.seconds)
    }
}


open class DecodeAutoStrategy(alliance: Alliance, val positions: String, vararg spikes: Spike) : PhasedAutonomous(alliance, phases {
    val (startingPoint, shootingPoint) = positions.split(",").map { it.trim() }

    autoStrategy(tilePose(startingPoint), tilePose(shootingPoint), *spikes)
}) {
    override fun init() {
        super.init()

        telemetry.addLine("Ensure robot is in position: ${positions.takeWhile { it != ',' }}")
    }
}

private fun PhaseBuilder.autoStrategy(startingPoint: Pose2D,
                                      launchPoint: Pose2D,
                                      vararg spikes: Spike) {
    assumePosition(startingPoint)
    shootInitialLoad(launchPoint)
    scoopAndShoot(spikes[0], launchPoint)
    scoopSpike(spikes[1])

}

private fun PhaseBuilder.shootInitialLoad(launchPose: Pose2D) {
    action {
        intake.enable(IntakeMode.ON)
        shooter.enableFlywheel(true)
    }
    driveTo(launchPose)
    wait(1.seconds)
    waitForDistance()
    action {
        shooter.launch()
        shooter.defaultGoalDistance = savedGoalDistanceCM
    }
    wait(ShooterConfig.SHOOTING_TIME_SECONDS.seconds)
}
