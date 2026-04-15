package org.beargineers.platform

/**
 * Provides relative positioning information - the movement/shift of the robot
 * since the last query. This is typically implemented using odometry sensors
 * like wheel encoders or dedicated odometry pods.
 *
 * Relative localization accumulates errors over time but provides continuous
 * updates regardless of environmental conditions.
 */
interface RelativeLocalizer {
    fun getPosition(oldPosition: Position): Position

    fun updatePositionEstimate(position: Position)

    /**
     * Returns movement delta normalized by time elapsed.
     */
    fun getVelocity(): Position
}

/**
 * Provides absolute positioning information - the robot's current position
 * on the field in global coordinates. This is typically implemented using
 * external references like AprilTags, GPS, or pre-mapped landmarks.
 *
 * Absolute localization does not accumulate errors but may be intermittent
 * (e.g., when AprilTags are not visible).
 */
interface AbsoluteLocalizer {
    /**
     * Returns the robot's current absolute position on the field with confidence.
     * Returns null if position cannot be determined (e.g., no AprilTag visible).
     *
     * @return AbsolutePose containing position
     *         or null if absolute position is unavailable or confidence is too low.
     */
    fun getRobotPose(): Position?
}

interface Localizer {
    fun setStartingPosition(position: Position)
    fun update()

    fun getPosition(): Position
    fun getVelocity(): Position
}

class FusionLocalizer(
    val absoluteLocalizer: AbsoluteLocalizer,
    val relativeLocalizer: RelativeLocalizer
) : Localizer {
    override fun setStartingPosition(position: Position) {
        updateCurrentPosition(position)
        relativeLocalizer.updatePositionEstimate(position)
    }

    private fun updateCurrentPosition(cp: Position) {
        RobotOpMode.lastKnownPosition = cp
    }

    override fun update() {
        val rel = relativeLocalizer.getPosition(RobotOpMode.lastKnownPosition)
        val abs = absoluteLocalizer.getRobotPose()
        if (abs != null) {
            Frame.addData("Vision", "✓ acquired")
            val res = abs.location().withHeading(rel.heading)
            relativeLocalizer.updatePositionEstimate(res)
            updateCurrentPosition(res)
        }
        else {
            Frame.addData("Vision", "✗ odometry only")
            updateCurrentPosition(rel)
        }
    }

    override fun getPosition(): Position {
        return RobotOpMode.lastKnownPosition
    }

    override fun getVelocity(): Position {
        return relativeLocalizer.getVelocity()
    }
}