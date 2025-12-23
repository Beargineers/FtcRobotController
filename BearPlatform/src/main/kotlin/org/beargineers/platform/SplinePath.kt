@file:Suppress("unused")

package org.beargineers.platform

import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Represents a smooth spline path with waypoints for robot navigation.
 *
 * The path is automatically optimized for speed using Catmull-Rom splines, which provide:
 * - Smooth curves through waypoints
 * - Continuous velocity (no sudden direction changes)
 * - Natural tension control for optimal speed
 *
 * Waypoints are treated as guide points rather than strict requirements, allowing
 * the spline to create a smooth, fast path that generally follows the intended direction.
 * Each waypoint includes heading, allowing precise control of robot orientation along the path.
 *
 * @param waypoints List of intermediate waypoints with positions and headings
 * @param target Final target position (location + heading)
 * @param startPosition Starting position (defaults to current robot position)
 * @param tension Spline tension (0.0 = tight curves, 1.0 = loose curves). Default 0.5.
 * @param resolution Number of points to generate per segment. Default 20.
 */
data class SplinePath(
    val waypoints: List<Position>,
    val target: Position,
    val startPosition: Position? = null,
    val tension: Double = 0.5,
    val resolution: Int = 20
) {
    /**
     * Generated spline points along the path, calculated lazily
     */
    val pathPoints: List<PathPoint> by lazy {
        generateSplinePoints()
    }

    /**
     * Total path length in distance units
     */
    val totalLength: Distance by lazy {
        pathPoints.zipWithNext().sumOf { (a, b) ->
            a.location.distanceTo(b.location).cm()
        }.cm
    }

    private fun generateSplinePoints(): List<PathPoint> {
        // Build control points list: start -> waypoints -> target
        val controlPoints = mutableListOf<Position>()

        // Add start point
        if (startPosition != null) {
            controlPoints.add(startPosition)
        }

        // Add waypoints
        controlPoints.addAll(waypoints)

        // Add target
        controlPoints.add(target)

        if (controlPoints.size < 2) {
            // Not enough points for a spline, just return straight line
            return listOf(
                PathPoint(controlPoints[0].location(), 0.cm, controlPoints[0].heading, 0.0)
            )
        }

        // Generate Catmull-Rom spline points
        val splinePoints = mutableListOf<PathPoint>()
        var cumulativeDistance = 0.0.cm

        // For Catmull-Rom, we need 4 control points per segment:
        // p0, p1, p2, p3 where we interpolate between p1 and p2
        // We duplicate first and last points to handle endpoints
        val extendedPoints = buildList {
            add(controlPoints.first())  // Duplicate first
            addAll(controlPoints)       // Add all N points
            add(controlPoints.last())   // Duplicate last
        }
        // extendedPoints now has N+2 elements at indices 0..(N+1)

        // We have N-1 segments to generate (between consecutive controlPoints)
        // Segment i interpolates between extendedPoints[i+1] and extendedPoints[i+2]
        for (i in 0 until controlPoints.size - 1) {
            val p0 = extendedPoints[i]      // i ranges 0 to N-2
            val p1 = extendedPoints[i + 1]  // ranges 1 to N-1
            val p2 = extendedPoints[i + 2]  // ranges 2 to N
            val p3 = extendedPoints[i + 3]  // ranges 3 to N+1 ✓ (max index is N+1)

            // Generate points for this segment
            for (j in 0..resolution) {
                val t = j.toDouble() / resolution
                val point = catmullRomPoint(p0.location(), p1.location(), p2.location(), p3.location(), t, tension)

                // Calculate heading - interpolate between waypoint headings
                // We're interpolating between p1 and p2 (the actual segment)
                val startH = p1.heading.radians()
                val endH = p2.heading.radians()
                val headingDiff = normalizeAngle(endH - startH)
                val heading = (startH + headingDiff * t).radians

                // Calculate curvature for speed optimization
                val curvature = calculateCurvature(p0.location(), p1.location(), p2.location(), p3.location(), t)

                splinePoints.add(PathPoint(point, cumulativeDistance, heading, curvature))

                // Update cumulative distance for next point
                if (splinePoints.size > 1) {
                    val prev = splinePoints[splinePoints.size - 2]
                    cumulativeDistance += point.distanceTo(prev.location)
                }
            }
        }

        return splinePoints
    }

    /**
     * Catmull-Rom spline interpolation
     * @param p0, p1, p2, p3 Four control points
     * @param t Parameter [0, 1] along the segment between p1 and p2
     * @param tension Tension parameter (0.5 = standard Catmull-Rom)
     */
    private fun catmullRomPoint(
        p0: Location, p1: Location, p2: Location, p3: Location,
        t: Double, tension: Double
    ): Location {
        val t2 = t * t
        val t3 = t2 * t

        val s = (1.0 - tension) / 2.0

        // Catmull-Rom basis matrix coefficients
        val c0 = -s * t3 + 2.0 * s * t2 - s * t
        val c1 = (2.0 - s) * t3 + (s - 3.0) * t2 + 1.0
        val c2 = (s - 2.0) * t3 + (3.0 - 2.0 * s) * t2 + s * t
        val c3 = s * t3 - s * t2

        val x = c0 * p0.x.cm() + c1 * p1.x.cm() + c2 * p2.x.cm() + c3 * p3.x.cm()
        val y = c0 * p0.y.cm() + c1 * p1.y.cm() + c2 * p2.y.cm() + c3 * p3.y.cm()

        return Location(x.cm, y.cm)
    }

    /**
     * Calculate curvature at a point (1/radius)
     * Higher curvature = tighter turn = slower speed needed
     */
    private fun calculateCurvature(
        p0: Location, p1: Location, p2: Location, p3: Location,
        t: Double
    ): Double {
        val epsilon = 0.001

        // Get point and nearby point to estimate derivative
        val point = catmullRomPoint(p0, p1, p2, p3, t, tension)
        val nextPoint = catmullRomPoint(p0, p1, p2, p3, min(t + epsilon, 1.0), tension)

        val dx = nextPoint.x.cm() - point.x.cm()
        val dy = nextPoint.y.cm() - point.y.cm()

        if (abs(dx) < 1e-6 && abs(dy) < 1e-6) return 0.0

        // Approximate curvature as change in direction
        val dist = hypot(dx, dy)
        if (dist < 1e-6) return 0.0

        // Simple curvature approximation
        return abs(atan2(dy.cm, dx.cm).radians() - atan2(p2.y - p1.y, p2.x - p1.x).radians()) / dist
    }

    private fun normalizeAngle(angle: Double): Double {
        var a = angle
        while (a > kotlin.math.PI) a -= 2 * kotlin.math.PI
        while (a < -kotlin.math.PI) a += 2 * kotlin.math.PI
        return a
    }

    /**
     * Find the closest point on the path to a given location
     * @return PathPoint and its index in the pathPoints list
     */
    private fun findClosestPointIndex(location: Location): Int {
        var minDist = Double.MAX_VALUE
        var closestIdx = 0

        pathPoints.forEachIndexed { idx, point ->
            val dist = point.location.distanceTo(location).cm()
            if (dist < minDist) {
                minDist = dist
                closestIdx = idx
            }
        }

        return closestIdx
    }

    /**
     * Get path following information for the current robot location.
     * Returns both the closest point (for progress tracking) and lookahead point (for steering).
     *
     * @param currentLocation Current robot location
     * @param lookaheadDistance How far ahead to look for steering
     * @return PathFollowingState containing closest point and lookahead point
     */
    fun getFollowingState(currentLocation: Location, lookaheadDistance: Distance): PathFollowingState {
        val closestIdx = findClosestPointIndex(currentLocation)
        val closestPoint = pathPoints[closestIdx]

        // Look ahead from closest point
        var accumulatedDist = 0.0.cm
        val targetDist = lookaheadDistance

        for (i in closestIdx until pathPoints.size - 1) {
            val curr = pathPoints[i]
            val next = pathPoints[i + 1]
            val segmentDist = curr.location.distanceTo(next.location)

            if (accumulatedDist + segmentDist >= targetDist) {
                // Interpolate between curr and next
                val remaining = targetDist - accumulatedDist
                val ratio = remaining / segmentDist

                val x = curr.location.x + (next.location.x - curr.location.x) * ratio
                val y = curr.location.y + (next.location.y - curr.location.y) * ratio
                val heading = curr.heading + (next.heading - curr.heading) * ratio
                val curvature = curr.curvature + (next.curvature - curr.curvature) * ratio

                val lookaheadPoint = PathPoint(Location(x, y), curr.distanceAlongPath, heading, curvature)
                return PathFollowingState(closestPoint, lookaheadPoint)
            }

            accumulatedDist += segmentDist
        }

        // If we couldn't find a lookahead point, use the last point
        val lastPoint = pathPoints.last()
        return PathFollowingState(closestPoint, lastPoint)
    }
}

/**
 * Represents a point along the spline path
 */
data class PathPoint(
    val location: Location,
    val distanceAlongPath: Distance,
    val heading: Angle,
    val curvature: Double  // 1/radius, higher = sharper turn
)

/**
 * Contains all information needed for path following at the current moment
 */
data class PathFollowingState(
    val closestPoint: PathPoint,      // Closest point on path (for progress tracking)
    val lookaheadPoint: PathPoint     // Point to steer towards
)

/**
 * Manages velocity profile along a path with smooth acceleration/deceleration
 */
class VelocityProfile(
    private val path: SplinePath,
    private val maxSpeed: Double,
    private val maxAcceleration: Double,
    private val maxDeceleration: Double,
    private val curvatureSpeedFactor: Double = 0.5  // How much curvature affects speed
) {
    /**
     * Calculate target speed at a given point along the path
     * Accounts for:
     * - Acceleration from start
     * - Deceleration to end
     * - Curvature-based speed limits
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
     * Calculate smooth speed ramp for current conditions
     * @param currentSpeed Current robot speed
     * @param targetSpeed Desired speed from profile
     * @param deltaTime Time since last update (seconds)
     * @return New commanded speed
     */
    fun calculateSmoothedSpeed(currentSpeed: Double, targetSpeed: Double, deltaTime: Double): Double {
        val speedDiff = targetSpeed - currentSpeed

        return if (speedDiff > 0) {
            // Accelerating
            min(currentSpeed + maxAcceleration * deltaTime, targetSpeed)
        } else {
            // Decelerating
            max(currentSpeed + maxDeceleration * deltaTime, targetSpeed)
        }
    }
}
