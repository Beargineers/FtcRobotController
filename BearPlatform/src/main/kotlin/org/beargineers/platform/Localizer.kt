package org.beargineers.platform

import org.firstinspires.ftc.robotcore.external.Telemetry

/**
 * Provides relative positioning information - the movement/shift of the robot
 * since the last query. This is typically implemented using odometry sensors
 * like wheel encoders or dedicated odometry pods.
 *
 * Relative localization accumulates errors over time but provides continuous
 * updates regardless of environmental conditions.
 */
interface RelativeLocalizer {
    fun getPosition(): Position

    fun updatePositionEstimate(position: Position)

    /**
     * Returns movement delta normalized by time elapsed.
     */
    fun getVelocity(): RelativePosition
}

/**
 * Represents an absolute position measurement with associated confidence.
 *
 * @property pose The robot's position on the field (x, y, heading)
 * @property confidence Quality metric from 0.0 (lowest) to 1.0 (highest) indicating
 *                      reliability of the position estimate. Factors affecting confidence:
 *                      - Distance to reference point (closer = higher confidence)
 *                      - Viewing angle (perpendicular = higher confidence)
 *                      - Number of features detected (more = higher confidence)
 *                      - Signal quality or detection ambiguity
 */
data class AbsolutePose(
    val pose: Position,
    val confidence: Double,
    val timestampNano: Long
) {
    init {
        require(confidence in 0.0..1.0) { "Confidence must be in range [0.0, 1.0], got $confidence" }
    }
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
    fun getVelocity(): RelativePosition
}

class FusionLocalizer(
    val telemetry: Telemetry,
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
        val pose = absoluteLocalizer.getRobotPose()
        if (pose != null) {
            telemetry.addData("Vision", "✓ acquired")
            relativeLocalizer.updatePositionEstimate(pose)
            updateCurrentPosition(pose)
        }
        else {
            telemetry.addData("Vision", "✗ odometry only")
            updateCurrentPosition(relativeLocalizer.getPosition())
        }
    }

    override fun getPosition(): Position {
        return RobotOpMode.lastKnownPosition
    }

    override fun getVelocity(): RelativePosition {
        return relativeLocalizer.getVelocity()
    }
}