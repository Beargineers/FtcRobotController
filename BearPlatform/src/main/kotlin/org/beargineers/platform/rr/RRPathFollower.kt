package org.beargineers.platform.rr

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.NullAction
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Position
import org.beargineers.platform.Waypoint
import org.beargineers.platform.atan2
import org.beargineers.platform.cm

class RRPathFollower(
    robot: BaseRobot,
    val path: List<Waypoint>,
    startPosition: Position
) {
    private val action: Action

    init {
        var hasMoves = false
        val movesBuilder = robot.mecanumDrive.movesBuilder(startPosition).apply {
            var prev = startPosition
            for (wp in path) {
                val next = wp.target
                if (next.distanceTo(prev) > 0.cm) {
                    val tangent = atan2(next.y - prev.y, next.x - prev.x)
                    setTangent(tangent)
                    splineToSplineHeading(next, tangent)
                    prev = next
                    hasMoves = true
                }
                else if (next.heading != prev.heading) {
                    turnTo(next.heading)
                    hasMoves = true
                }
            }
        }

        action = if (hasMoves) {
            movesBuilder.build()
        } else {
            NullAction()
        }
    }

    fun update(): Boolean {
        return action.run(TelemetryPacket())
    }
}