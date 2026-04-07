package org.beargineers.gamma

import org.beargineers.platform.AbsoluteLocalizer
import org.beargineers.platform.Position
import org.beargineers.platform.RobotCentricLocation
import org.beargineers.platform.cm
import org.beargineers.platform.toFieldCentric

class RotatingCameraAdjuster(val delegate: AbsoluteLocalizer, val turret: Turret) : AbsoluteLocalizer {
    override fun getRobotPose(): Position? {
        return delegate.getRobotPose()?.let {
            val turretCenter = Position(it.x, it.y, it.heading - turret.currentTurretAngle())
            return RobotCentricLocation(turret.centerOffset, 0.cm).toFieldCentric(turretCenter).withHeading(turretCenter.heading)
        }
    }
}