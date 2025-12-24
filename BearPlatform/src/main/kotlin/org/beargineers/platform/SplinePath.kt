@file:Suppress("unused")

package org.beargineers.platform

import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.min

/**
 * Immutable definition of a navigation path.
 *
 * A Path is just the route definition - it contains waypoints and spline parameters,
 * but no state. Create Path instances once (e.g., as properties) and reuse them.
 *
 * The path is automatically optimized for speed using Catmull-Rom splines, which provide:
 * - Smooth curves through waypoints
 * - Continuous velocity (no sudden direction changes)
 * - Natural tension control for optimal speed
 *
 * Waypoints are treated as guide points rather than strict requirements.
 * The last waypoint in the list is treated as the final target position.
 *
 * @param waypoints List of waypoints (positions + headings) to follow. Last waypoint is the target.
 *                  Must have at least 1 waypoint.
 * @param tension Spline tension (0.0 = tight curves, 1.0 = loose curves). Default 0.5.
 * @param resolution Number of points to generate per segment. Default 20.
 *
 * ## Example Usage
 * ```kotlin
 * // Define path once as property - it's immutable
 * val myPath = Path(listOf(
 *     Location(30.cm, 20.cm).withHeading(0.degrees),
 *     Location(100.cm, 100.cm).withHeading(90.degrees)
 * ))
 *
 * // Follow in loop
 * override fun loop() {
 *     super.loop()
 *     if (followPath(myPath, maxSpeed = 0.8)) {
 *         telemetry.addLine("Following path...")
 *     }
 * }
 * ```
 */
data class Path(
    val waypoints: List<Position>,
    val tension: Double = 0.5,
    val resolution: Int = 20
) {
    init {
        require(waypoints.isNotEmpty()) { "Waypoints list must contain at least one position (the target)" }
    }

    /**
     * The target position (last waypoint in the list)
     */
    val target: Position get() = waypoints.last()

    /**
     * Generate spline points for this path given a starting position.
     *
     * @param startPosition Where the robot starts following this path
     * @return Generated spline representation
     */
    internal fun generateSplineRepresentation(startPosition: Position): SplineRepresentation {
        // Build control points list: start -> waypoints (including target as last waypoint)
        val controlPoints = listOf(startPosition) + waypoints

        if (controlPoints.size < 2) {
            // Not enough points for a spline, just return straight line
            val singlePoint =
                PathPoint(controlPoints[0].location(), 0.cm, controlPoints[0].heading, 0.0)
            return SplineRepresentation(listOf(singlePoint), 0.cm)
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
                val point = catmullRomPoint(
                    p0.location(),
                    p1.location(),
                    p2.location(),
                    p3.location(),
                    t,
                    tension
                )

                // Calculate heading - interpolate between waypoint headings
                // We're interpolating between p1 and p2 (the actual segment)
                val startH = p1.heading.radians()
                val endH = p2.heading.radians()
                val headingDiff = normalizeAngle(endH - startH)
                val heading = (startH + headingDiff * t).radians

                // Calculate curvature for speed optimization
                val curvature = calculateCurvature(
                    p0.location(),
                    p1.location(),
                    p2.location(),
                    p3.location(),
                    t,
                    tension
                )

                val prev = splinePoints.lastOrNull()

                if (prev != null) {
                    val distance = prev.location.distanceTo(point)
                    cumulativeDistance += distance
                    if (distance > 0.1.cm) {
                        splinePoints.add(PathPoint(point, cumulativeDistance, heading, curvature))
                    }
                }
                else {
                    splinePoints.add(PathPoint(point, cumulativeDistance, heading, curvature))
                }
            }
        }

        // Calculate total length
        val totalLength = splinePoints.zipWithNext().sumOf { (a, b) ->
            a.location.distanceTo(b.location).cm()
        }.cm

        return SplineRepresentation(splinePoints, totalLength)
    }

}

/**
 * Generated spline representation with points and metadata.
 */
internal data class SplineRepresentation(
    val pathPoints: List<PathPoint>,
    val totalLength: Distance
)

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
    t: Double, tension: Double
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
internal data class PathFollowingState(
    val closestPoint: PathPoint,      // Closest point on path (for progress tracking)
    val closestPointIndex: Int,       // Index of closest point (for next iteration)
    val lookaheadPoint: PathPoint     // Point to steer towards
)
