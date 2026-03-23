package org.beargineers.straffer

import org.beargineers.platform.AbsoluteLocalizer
import org.beargineers.platform.Angle
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.FusionLocalizer
import org.beargineers.platform.Localizer
import org.beargineers.platform.PinpointLocalizer
import org.beargineers.platform.Position
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.degrees

object NoVision : AbsoluteLocalizer {
    override fun getRobotPose(): Position? {
        return null
    }
}

class StrafferRobot(op: RobotOpMode<DecodeRobot>) : BaseRobot(op), DecodeRobot {
    val vision = ArtifactsVision(this)

    override val localizer: Localizer = FusionLocalizer(NoVision, PinpointLocalizer(this))
    override val hasTurret = true

    override fun adjustShooting(distance: Double, angle: Double) {

    }

    override val intakeMode: IntakeMode get() = IntakeMode.OFF
    override val shootingAngleCorrection: Angle
        get() = 0.degrees
    override val artifactsCount: Int
        get() = 0

    override fun intakeMode(mode: IntakeMode) {
        // TODO("Not yet implemented")
    }

    override fun launch() {
        // TODO("Not yet implemented")
    }

    override fun prepareForShooting() {
        // TODO("Not yet implemented")
    }

    override fun enableFlywheel(on: Boolean) {
        // TODO("Not yet implemented")
    }

    override fun isShooting(): Boolean {
        return false
    }

    override fun warnDriverON() {
    }

}