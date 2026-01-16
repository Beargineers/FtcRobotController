package org.beargineers.platform

data class Waypoint(
    val target: Position,
    val speed: Double = 1.0,
    val onArrival: () -> Unit = {}
)

fun <T: Robot> T.followPath(path: List<Waypoint>): Boolean {
    return opMode.followPath(path)
}