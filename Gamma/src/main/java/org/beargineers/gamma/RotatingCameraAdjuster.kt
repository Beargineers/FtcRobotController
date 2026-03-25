package org.beargineers.gamma

import org.beargineers.platform.AbsoluteLocalizer
import org.beargineers.platform.Position

class RotatingCameraAdjuster(val delegate: AbsoluteLocalizer, val turret: Turret) : AbsoluteLocalizer {
    override fun getRobotPose(): Position? {
        return delegate.getRobotPose()?.let {
            // TODO: Adjust x and y too having in mind the center of the turret is not aligned with robot's center
            Position(it.x, it.y, it.heading - turret.currentTurretAngle())
        }
    }
}