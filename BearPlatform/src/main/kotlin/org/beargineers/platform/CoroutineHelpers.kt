package org.beargineers.platform

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.beargineers.platform.rr.RRPathFollower

suspend fun Robot.drivePath(waypoints: List<Waypoint>) {
    val follower = RRPathFollower(
        robot = this as BaseRobot,
        path = waypoints,
        startPosition = currentPosition
    )

    while (follower.update()) {
        opMode.loop.nextTick()
    }
}

suspend fun Robot.driveTo(target: Position) {
    drivePath(pathTo(target))
}

suspend fun Robot.driveRelative(movement: RobotCentricPosition) {
    driveTo(currentPosition + movement)
}

suspend fun Robot.doWhile(condition: () -> Boolean, block: suspend CoroutineScope.() -> Unit) {
    coroutineScope {
        val childJob = launch(block = block)
        while (childJob.isActive && condition()) {
            opMode.loop.nextTick()
        }

        if (childJob.isActive) {
            childJob.cancel()
        }
    }
}