package org.beargineers.beta

import org.beargineers.platform.AbsoluteLocalizer
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.LimelightCam
import org.beargineers.platform.MecanumDrive
import org.beargineers.platform.PinpointLocalizer
import org.beargineers.platform.RelativeLocalizer
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.IntakeMode

class BetaRobot(op: RobotOpMode<DecodeRobot>) : BaseRobot(op), DecodeRobot {
    override val drive = MecanumDrive(this)

    override val absoluteLocalizer: AbsoluteLocalizer = LimelightCam(this)
    override val relativeLocalizer: RelativeLocalizer = PinpointLocalizer(this)

    override val configResource: Int = R.raw.config

    val shooter = Shooter(this)

    val intake = Intake(this)
    override val intakeMode: IntakeMode get() = intake.mode
    override fun intakeMode(mode: IntakeMode) {
        intake.mode = mode
    }

    override fun launch() {
        shooter.launch()
    }

    override fun enableFlywheel(on: Boolean) {
        shooter.enableFlywheel(on)
    }

    override fun isShooting(): Boolean {
        return shooter.feederStartedAt != 0L
    }
}