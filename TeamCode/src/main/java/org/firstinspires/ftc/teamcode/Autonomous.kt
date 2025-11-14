package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime


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

class TurnPhase(val degrees: Int, val power: Double) : AutonomousPhase {
    override fun Robot.initPhase() {
        // Initialize the turn using the drivetrain's turn function
        // This sets motor targets for the rotation
        drive.turn(degrees, power)
    }

    override fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean {
        // Continue the phase while any motor is still busy turning
        // Returns true to keep running, false when all motors are done
        return drive.allMotors.any() { it.isBusy }
    }
}

class GoPhase(val ycm: Double, val xcm: Double, val power: Double): AutonomousPhase {
    override fun Robot.initPhase() {
        // Calculate and set motor targets for the desired movement
        drive.goto(ycm, xcm, power)
    }

    override fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean {
        // Continue the phase while any motor is still busy moving
        // Returns true to keep running, false when all motors reach their targets
        return drive.allMotors.any() { it.isBusy }
    }
}

@Autonomous(name = "Simple Autonomous")
class SimpleAutonomous : PhasedAutonomous(
    phases {
        phase(GoPhase(12.0, 0.0, 0.5))
        phase(TurnPhase(180, 0.5))
        phase(GoPhase(12.0, 0.0, 0.5))
    }
)

@Autonomous(name = "Red Alliance South")
class RedSouthAutonomous : PhasedAutonomous(phases("Autonomous") {
    scoopAndShoot(Spike.RIGHT1, SOUTH_RED_LAUNCH_POINT)
    scoopAndShoot(Spike.RIGHT2, SOUTH_RED_LAUNCH_POINT)
    scoopAndShoot(Spike.RIGHT3, SOUTH_RED_LAUNCH_POINT)
})