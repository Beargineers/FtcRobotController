@file:Suppress("unused")

package org.beargineers.platform

/**
 * Manages the state and execution of following a specific Path.
 *
 * A PathFollower is created when you start following a new Path instance.
 * It holds all the state required for following (progress, speed, generated spline, etc.).
 *
 * Each PathFollower is bound to a specific Path and starting position.
 *
 * @param path The immutable Path definition to follow
 * @param robot The robot instance to control
 * @param startPosition Where the robot starts following this path
 * @param maxSpeed Maximum speed for this path execution
 * @param kP_position Proportional gain for position control
 * @param kP_heading Proportional gain for heading control
 * @param minimalWheelPower Minimum wheel power threshold
 * @param positionTolerance Position tolerance for target reached (cm)
 * @param headingTolerance Heading tolerance for target reached (degrees)
 */
internal class PathFollower(
    val path: List<Position>,
    private val robot: Robot,
    startPosition: Position,
    private val kP_position: Double,
    private val kP_heading: Double,
    private val minimalWheelPower: Double,
    private val positionTolerance: Double,
    private val headingTolerance: Double
) {
    private var lastTargetIndex: Int = 0
    private var currentAngle = atan2(path.first().y - startPosition.y, path.first().x - startPosition.x)
    private var currentSpeed: Double = 0.0
    private var lastUpdateNanos: Long = System.nanoTime()

    /**
     * Execute one update cycle of path following.
     * Calculates drive powers and commands the robot directly.
     *
     * @return true if still following, false if target reached
     */
    fun update(): Boolean {
        if (lastTargetIndex > path.lastIndex) return false

        val headingToTarget = atan2(path[lastTargetIndex].y - robot.currentPosition.y, path[lastTargetIndex].x - robot.currentPosition.x)
        if (abs((headingToTarget - currentAngle).normalize()) > 100.degrees && lastTargetIndex < path.lastIndex) {
            // Angle to the target changes quickly means we're passed the waypoint
            lastTargetIndex++
            currentAngle = atan2(path[lastTargetIndex].y - path[lastTargetIndex - 1].y, path[lastTargetIndex].x - path[lastTargetIndex - 1].x)
        }

        // Calculate time delta
        val currentNanos = System.nanoTime()
        val deltaTime = (currentNanos - lastUpdateNanos) / 1e6
        lastUpdateNanos = currentNanos

        // Get path following state
        val currentPosition = robot.currentPosition
        val currentLocation = currentPosition.location()

        val currentTarget = path[lastTargetIndex]

        val distanceToTarget = currentLocation.distanceTo(currentTarget.location())
        val headingError = (currentTarget.heading - currentPosition.heading).normalize()

        val finished = lastTargetIndex == path.lastIndex &&
                distanceToTarget.cm() < positionTolerance &&
                abs(headingError).degrees() < headingTolerance


        if (finished) {
            robot.stopDriving()
            return false
        }

        // Calculate position error (field frame)
        val deltaX = currentTarget.x - currentPosition.x
        val deltaY = currentTarget.y - currentPosition.y

        // Convert field-frame error to robot-frame error
        val robotHeadingRad = currentPosition.heading
        val robotForward = deltaX * cos(robotHeadingRad) + deltaY * sin(robotHeadingRad)
        val robotRight = deltaX * sin(robotHeadingRad) - deltaY * cos(robotHeadingRad)

        // Calculate drive powers - scale by current path speed
        val speedScale = if (lastTargetIndex == path.lastIndex) {
            1.0
        }
        else {
            val headingDiff = abs(atan2(path[lastTargetIndex + 1].y - currentTarget.y, path[lastTargetIndex + 1].x - currentTarget.x) - currentAngle).degrees()
            (2 - headingDiff/180)
        }

        val forwardPower = robotForward.cm() * kP_position * speedScale
        val strafePower = robotRight.cm() * kP_position * speedScale
        val turnPower = -headingError.degrees() * kP_heading * speedScale

        val maxPower = listOf(kotlin.math.abs(forwardPower), kotlin.math.abs(strafePower), kotlin.math.abs(turnPower)).maxOf { it }
        val maxV = when {
            maxPower > 1.0 -> maxPower
            maxPower < minimalWheelPower && maxPower > 1e-6 -> maxPower / minimalWheelPower
            else -> 1.0
        }

        fun clamp(value: Double): Double {
            return if (maxV > 1e-6) value / maxV else 0.0
        }

        // Apply drive power to robot
        robot.drive(
            forwardPower = clamp(forwardPower),
            rightPower = clamp(strafePower),
            turnPower = clamp(turnPower)
        )

        return true
    }
}