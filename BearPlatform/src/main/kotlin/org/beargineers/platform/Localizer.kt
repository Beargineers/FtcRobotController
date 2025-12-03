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
    /**
     * Returns the robot's movement since the last call to this method.
     * This includes forward/right displacement and rotation.
     *
     * Each call updates internal state, so calling this method twice
     * in succession will return different deltas (second call returns
     * movement since first call).
     *
     * @return RobotMovement containing forward, right, and turn deltas
     */
    fun getMovementDelta(): RelativePosition

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
     * Multiple calls to this method return the current position estimate
     * at each point in time - it does not modify internal state.
     *
     * Confidence indicates measurement quality:
     * - 1.0: Excellent (close range, good angle, clear detection)
     * - 0.7-0.9: Good (moderate range/angle)
     * - 0.4-0.6: Fair (far range or poor angle)
     * - 0.0-0.3: Poor (very far, extreme angle, or ambiguous)
     *
     * @return AbsolutePose containing position and confidence,
     *         or null if absolute position is unavailable
     */
    fun getRobotPose(): AbsolutePose?
}
