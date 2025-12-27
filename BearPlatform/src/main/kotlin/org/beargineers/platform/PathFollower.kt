@file:Suppress("unused")

package org.beargineers.platform

import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

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
    val path: Path,
    private val robot: Robot,
    startPosition: Position,
    private val maxSpeed: Double,
    private val kP_position: Double,
    private val kP_heading: Double,
    private val minimalWheelPower: Double,
    private val positionTolerance: Double,
    private val headingTolerance: Double
) {
    // Configuration parameters
    val lookaheadDistance by robot.config(5.0) // cm
    val maxPathAcceleration by robot.config(50.0) // cm/s²
    val maxPathDeceleration by robot.config(60.0) // cm/s²
    val curvatureSpeedFactor by robot.config(30.0) // cm

    // Generated spline representation (immutable once created)
    private val spline: SplineRepresentation = path.generateSplineRepresentation(startPosition)

    // Velocity profile for this path
    private val velocityProfile: VelocityProfile = VelocityProfile(
        path = path,
        spline = spline,
        maxSpeed = maxSpeed,
        maxAcceleration = maxPathAcceleration,
        maxDeceleration = maxPathDeceleration,
        curvatureSpeedFactor = curvatureSpeedFactor
    )

    // Following state (mutable, updated each loop)
    private var lastClosestPointIndex: Int = 0
    private var currentSpeed: Double = 0.0
    private var lastUpdateNanos: Long = System.nanoTime()

    /**
     * Get the current following state for the robot's location.
     *
     * @param currentLocation Current robot location
     * @return PathFollowingState with closest point and lookahead point
     */
    fun getFollowingState(currentLocation: Location): PathFollowingState {
        // Only search forward from last known position
        val searchStart = lastClosestPointIndex

        // Limit search window based on lookahead distance to prevent jumping ahead
        // Search 3x lookahead distance worth of path points
        val avgPointSpacing = if (spline.pathPoints.size > 1) {
            spline.totalLength.cm() / spline.pathPoints.size
        } else {
            10.0
        }
        val maxSearchPoints = ((lookaheadDistance.cm * 3.0) / avgPointSpacing).cm().toInt().coerceAtLeast(30)
        val searchWindow = min(maxSearchPoints, spline.pathPoints.size - searchStart)

        val closestIdx = findClosestPointIndex(currentLocation, searchStart, searchWindow)
        val closestPoint = spline.pathPoints[closestIdx]

        // Look ahead from closest point
        var accumulatedDist = 0.0.cm
        val targetDist = lookaheadDistance.cm

        for (i in closestIdx until spline.pathPoints.size - 1) {
            val curr = spline.pathPoints[i]
            val next = spline.pathPoints[i + 1]
            val segmentDist = curr.location.distanceTo(next.location)

            if (accumulatedDist + segmentDist >= targetDist) {
                // Interpolate between curr and next
                val remaining = targetDist - accumulatedDist
                val ratio = remaining / segmentDist

                val x = curr.location.x + (next.location.x - curr.location.x) * ratio
                val y = curr.location.y + (next.location.y - curr.location.y) * ratio
                val distance = curr.distanceAlongPath + (next.distanceAlongPath - curr.distanceAlongPath) * ratio
                val heading = curr.heading + (next.heading - curr.heading) * ratio
                val curvature = curr.curvature + (next.curvature - curr.curvature) * ratio

                val lookaheadPoint = PathPoint(Location(x, y), distance, heading, curvature)
                return PathFollowingState(closestPoint, closestIdx, lookaheadPoint)
            }

            accumulatedDist += segmentDist
        }

        // If we couldn't find a lookahead point, use the last point
        val lastPoint = spline.pathPoints.last()
        return PathFollowingState(closestPoint, closestIdx, lastPoint)
    }

    /**
     * Execute one update cycle of path following.
     * Calculates drive powers and commands the robot directly.
     *
     * @return true if still following, false if target reached
     */
    fun update(): Boolean {
        // Calculate time delta
        val currentNanos = System.nanoTime()
        val deltaTime = (currentNanos - lastUpdateNanos) / 1e6
        lastUpdateNanos = currentNanos

        // Get path following state
        val currentPosition = robot.currentPosition
        val currentLocation = currentPosition.location()
        val followingState = getFollowingState(currentLocation)

        val closestPoint = followingState.closestPoint
        val lookaheadPoint = followingState.lookaheadPoint

        // Update progress tracking
        lastClosestPointIndex = followingState.closestPointIndex

        // Calculate remaining distance to target
        val remainingDistance = spline.totalLength - closestPoint.distanceAlongPath + closestPoint.location.distanceTo(currentLocation)
        val progress = closestPoint.distanceAlongPath / spline.totalLength

        // Check if we've reached the target
        val target = path.target
        val distanceToTarget = currentLocation.distanceTo(target.location())
        val headingError = (target.heading - currentPosition.heading).normalize()

        val finished = progress > 0.9 &&
                distanceToTarget.cm() < positionTolerance &&
                abs(headingError).degrees() < headingTolerance

        // Calculate target speed based on velocity profile
        val targetSpeed = velocityProfile.getTargetSpeed(lookaheadPoint, remainingDistance)
        currentSpeed = velocityProfile.calculateSmoothedSpeed(currentSpeed, targetSpeed, deltaTime)

        robot.telemetry.addData("Path Progress", "%.1f%%", 100.0 * progress)
        robot.telemetry.addData("Remaining", remainingDistance)
        robot.telemetry.addData("Path Speed", "%.2f (target: %.2f)", currentSpeed, targetSpeed)
        robot.telemetry.addData("Curvature", "%.4f", lookaheadPoint.curvature)

        if (finished) {
            robot.stopDriving()
            return false
        }

        // Pure pursuit: drive toward lookahead point with speed scaling
        val targetPosition = Position(
            lookaheadPoint.location.x,
            lookaheadPoint.location.y,
            lookaheadPoint.heading
        )

        // Calculate position error (field frame)
        val deltaX = targetPosition.x - currentPosition.x
        val deltaY = targetPosition.y - currentPosition.y

        // Calculate heading error
        val headingErr = (targetPosition.heading - currentPosition.heading).normalize()

        // Convert field-frame error to robot-frame error
        val robotHeadingRad = currentPosition.heading
        val robotForward = deltaX * cos(robotHeadingRad) + deltaY * sin(robotHeadingRad)
        val robotRight = deltaX * sin(robotHeadingRad) - deltaY * cos(robotHeadingRad)

        // Calculate drive powers - scale by current path speed
        val speedScale = currentSpeed / maxSpeed
        val forwardPower = robotForward.cm() * kP_position * 2.2 * speedScale
        val strafePower = robotRight.cm() * kP_position * 2.2 * speedScale
        val turnPower = -headingErr.degrees() * kP_heading * 2.2 * speedScale

        val maxPower = listOf(kotlin.math.abs(forwardPower), kotlin.math.abs(strafePower), kotlin.math.abs(turnPower)).maxOf { it }
        val maxV = when {
            maxPower > maxSpeed -> maxPower / maxSpeed
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

    /**
     * Find the closest point on the path, searching forward from a starting index.
     */
    private fun findClosestPointIndex(
        location: Location,
        startIdx: Int = 0,
        searchWindow: Int = spline.pathPoints.size
    ): Int {
        var minDist = Double.MAX_VALUE
        var closestIdx = startIdx

        // Search forward from startIdx
        val endIdx = min(startIdx + searchWindow, spline.pathPoints.size)

        for (idx in startIdx until endIdx) {
            val point = spline.pathPoints[idx]
            val dist = point.location.distanceTo(location).cm()

            if (dist < minDist) {
                minDist = dist
                closestIdx = idx
            }
        }

        // Ensure we always progress forward (or stay at current index)
        return max(closestIdx, startIdx)
    }
}

/**
 * Manages velocity profile along a path with smooth acceleration/deceleration.
 */
internal class VelocityProfile(
    private val path: Path,
    private val spline: SplineRepresentation,
    private val maxSpeed: Double,
    private val maxAcceleration: Double,
    private val maxDeceleration: Double,
    private val curvatureSpeedFactor: Double = 0.5
) {
    /**
     * Calculate target speed at a given point along the path.
     */
    fun getTargetSpeed(point: PathPoint, remainingDistance: Distance): Double {
        // Speed limit based on curvature (tighter turns = slower)
        val curvatureLimit = if (point.curvature > 1e-6) {
            min(maxSpeed, curvatureSpeedFactor / point.curvature)
        } else {
            maxSpeed
        }

        // Speed limit based on acceleration from start
        val distanceFromStart = point.distanceAlongPath.cm()
        val accelLimit = sqrt(2.0 * maxAcceleration * max(0.0, distanceFromStart))

        // Speed limit based on deceleration to end
        val distToEnd = remainingDistance.cm()
        val decelLimit = sqrt(2.0 * maxDeceleration * max(0.0, distToEnd))

        // Take minimum of all limits
        return min(min(min(curvatureLimit, accelLimit), decelLimit), maxSpeed)
    }

    /**
     * Calculate smooth speed ramp for current conditions.
     */
    fun calculateSmoothedSpeed(currentSpeed: Double, targetSpeed: Double, deltaTime: Double): Double {
        val speedDiff = targetSpeed - currentSpeed

        return if (speedDiff > 0) {
            // Accelerating
           min(maxSpeed, max(currentSpeed + maxAcceleration * deltaTime, targetSpeed))
        } else {
            // Decelerating
            min(maxSpeed, min(currentSpeed + maxDeceleration * deltaTime, targetSpeed))
        }
    }
}
