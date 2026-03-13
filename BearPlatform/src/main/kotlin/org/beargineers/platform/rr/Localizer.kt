package org.beargineers.platform.rr

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d

/**
 * Interface for localization methods.
 */
 interface Localizer {
    fun setPose(pose: Pose2d)

    /**
     * Returns the current pose estimate.
     * NOTE: Does not update the pose estimate;
     * you must call update() to update the pose estimate.
     * @return the Localizer's current pose
     */
    fun getPose(): Pose2d

    /**
     * Updates the Localizer's pose estimate.
     * @return the Localizer's current velocity estimate
     */
    fun update(): PoseVelocity2d
}
