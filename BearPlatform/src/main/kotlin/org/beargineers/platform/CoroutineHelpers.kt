package org.beargineers.platform

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch

suspend fun Robot.drivePath(waypoints: List<Waypoint>) {
    var hasMoves = false
    val movesBuilder = (this as BaseRobot).mecanumDrive.movesBuilder(currentPosition).apply {
        var prev = currentPosition
        for (wp in waypoints) {
            val next = wp.target
            if (next.distanceTo(prev) > 0.cm) {
                val tangent = atan2(next.y - prev.y, next.x - prev.x)
                setTangent(tangent)
                splineToSplineHeading(next, tangent)
                hasMoves = true
            }
            else if (next.heading != prev.heading) {
                turnTo(next.heading)
                hasMoves = true
            }
            prev = next
        }
    }

    if (hasMoves) {
        val action = movesBuilder.build()
        while (action.run(TelemetryPacket())) {
            opMode.loop.nextTick()
        }
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