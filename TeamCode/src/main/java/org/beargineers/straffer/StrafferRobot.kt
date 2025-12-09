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
import org.beargineers.robot.WheelCorrections

class StrafferRobot(op: RobotOpMode<StrafferRobot>) : BaseRobot(op) {
    override val drive = MecanumDrive(this, WheelCorrections.asConfig())
    private val lime = LimelightCam(this)
    private val pinpoint = PinpointLocalizer(this)

    override val absoluteLocalizer: AbsoluteLocalizer get() = lime
    override val relativeLocalizer: RelativeLocalizer get() = pinpoint

    override fun configureKalmanFilter(): KalmanFilter {
        return KalmanFilter()
    }

    override fun configureAutonomousDriving(): AutonomousDriveConfig {
        return AutonomousDriveConfig()
    }
}