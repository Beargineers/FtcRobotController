package org.beargineers.platform

import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineName
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.beargineers.platform.decode.mirrorForAlliance
import kotlin.time.Duration

private var driveJob: Job? = null
suspend fun Robot.drivePath(waypoints: List<Waypoint>, applyMirroring: Boolean, stopAtLastWaypoint: Boolean = true) {
    val waypoints = if (applyMirroring) {
        waypoints.map { it.copy(target = it.target.mirrorForAlliance(alliance)) }
    } else {
        waypoints
    }

    Frame.log("DRV") { "drivePath: $waypoints" }

    driveJob?.cancel()
    coroutineScope {
        driveJob = launch {
            val follower = PathFollower(
                robot = this@drivePath as BaseRobot,
                path = waypoints,
                startPosition = currentPosition,
                stopAtLastWaypoint = stopAtLastWaypoint
            )

            while (follower.update()) {
                nextTick()
            }
        }
    }
}

suspend fun Robot.driveTo(target: Position, speed: Double = 1.0, applyMirroring: Boolean, stopAtLastWaypoint: Boolean = true) {
    drivePath(pathTo(target, speed), applyMirroring, stopAtLastWaypoint)
}

suspend fun Robot.driveRelative(movement: RobotCentricPosition) {
    driveTo(currentPosition + movement, applyMirroring = false)
}

/**
 * Executes `block` once until it finishes normally or `condition` becomes false. In latter case, execution of `block` is aborted
 */
suspend fun Robot.cancelWhen(condition: () -> Boolean, block: suspend CoroutineScope.() -> Unit) {
    coroutineScope {
        val childJob = launch(block = block)
        while (childJob.isActive && !condition()) {
            nextTick()
        }

        if (childJob.isActive) {
            childJob.cancel()
        }
    }
}

suspend fun Robot.doNoLongerThan(duration: Duration, block: suspend CoroutineScope.() -> Unit) {
    val elapsed = ElapsedTime()
    cancelWhen({ elapsed.milliseconds() > duration.inWholeMilliseconds}, block)
}

suspend fun withName(name: String, block: suspend CoroutineScope.() -> Unit) {
    Frame.log("Doing $name")
    try {
        withContext(CoroutineName(name), block)
    } catch (e: CancellationException) {
        Frame.log("$name is cancelled")
        throw e
    }
    Frame.log("$name is finished")
}