package org.beargineers.platform

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.beargineers.platform.rr.MovesBuilder

suspend fun Robot.move(moves: MovesBuilder.() -> Unit) {
    val movesBuilder = (this as BaseRobot).mecanumDrive.movesBuilder(currentPosition)
    movesBuilder.apply(moves)
    val action = movesBuilder.build()
    while (action.run(TelemetryPacket())) {
        opMode.loop.nextTick()
    }
}

suspend fun Robot.drivePath(waypoints: List<Waypoint>) {
    move {
        var prev = currentPosition
        for (wp in waypoints) {
            val next = wp.target
            if (next.distanceTo(prev) > 0.cm) {
                val tangent = atan2(next.y - prev.y, next.x - prev.x)
                setTangent(tangent)
                splineToSplineHeading(next, tangent)
            }
            else if (next.heading != prev.heading) {
                turnTo(next.heading)
            }
            prev = next
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