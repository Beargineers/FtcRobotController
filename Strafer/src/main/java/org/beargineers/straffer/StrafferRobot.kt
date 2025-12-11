package org.beargineers.straffer

import org.beargineers.platform.AbsoluteLocalizer
import org.beargineers.platform.AutonomousDriveConfig
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.KalmanFilter
import org.beargineers.platform.LimelightCam
import org.beargineers.platform.MecanumDrive
import org.beargineers.platform.PinpointLocalizer
import org.beargineers.platform.RelativeLocalizer
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.WheelsConfig
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.IntakeMode

class StrafferRobot(op: RobotOpMode<DecodeRobot>) : BaseRobot(op), DecodeRobot {
    override val drive = MecanumDrive(this, WheelsConfig(cm_per_tick_strafe = 1.0, cm_per_tick_forward = 1.0))

    override val absoluteLocalizer: AbsoluteLocalizer = LimelightCam(this)
    override val relativeLocalizer: RelativeLocalizer = PinpointLocalizer(this)

    override fun configureKalmanFilter(): KalmanFilter {
        return KalmanFilter()
    }

    override fun configureAutonomousDriving(): AutonomousDriveConfig {
        return AutonomousDriveConfig()
    }

    override fun intakeMode(mode: IntakeMode) {
        // TODO("Not yet implemented")
    }

    override fun launch() {
        // TODO("Not yet implemented")
    }

    override fun enableFlywheel(on: Boolean) {
        // TODO("Not yet implemented")
    }

    override fun isShooting(): Boolean {
        return false
    }
}