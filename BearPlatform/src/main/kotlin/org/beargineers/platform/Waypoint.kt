package org.beargineers.platform

data class Waypoint(
    val target: Position,
    val speed: Double = 1.0,
    val positionTolerance: Distance?,
    val headingTolerance: Angle?
)

object PathFollowingConfig {
    val positionToleranceToStop by config(3.cm)
    val headingToleranceToStop by config(10.degrees)
    val positionTolerance by config(2.cm)
    val headingTolerance by config(1.degrees)
    val artifactStrafeTolerance by config(2.cm)

    val intermediateWaypointPositionTolerance by config(8.cm)
    val intermediateWaypointHeadingTolerance by config(5.degrees)

    val stalledPathAbortTimeoutMillis by config(100)
}

fun buildPath(block: PathBuilder.() -> Unit): List<Waypoint> {
    return PathBuilder().apply(block).build()
}

fun pathTo(
    target: Position,
    speed: Double = 1.0,
    positionTolerance: Distance? = null,
    headingTolerance: Angle? = null
): List<Waypoint> {
    return buildPath {
        addWaypoint(target, speed, positionTolerance, headingTolerance)
    }
}

class PathBuilder {
    private val result = mutableListOf<Waypoint>()

    fun addWaypoint(target: Position, speed: Double = 1.0, positionTolerance: Distance? = null, headingTolerance: Angle? = null) {
        result += Waypoint(target, speed, positionTolerance, headingTolerance)
    }

    fun addStrictHeadingWaypoint(target: Position, speed: Double = 1.0) {
        addWaypoint(target, positionTolerance = 5.cm, headingTolerance = 1.degrees)
    }

    fun addRelaxedWaypoint(target: Position, speed: Double = 1.0) {
        addWaypoint(target, positionTolerance = 5.cm, headingTolerance = 5.degrees)
    }

    fun build(): List<Waypoint> {
        return result
    }
}