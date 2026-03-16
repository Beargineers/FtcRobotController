package org.beargineers.platform

import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.rr.RRPathFollower

suspend fun Robot.s_followPath(waypoints: List<Waypoint>) {
    val follower = RRPathFollower(
        robot = this as BaseRobot,
        path = waypoints,
        startPosition = currentPosition
    )

    while (follower.update()) {
        opMode.loop.nextTick()
    }
}

suspend fun Robot.s_driveToTarget(target: Position) {
    s_followPath(pathTo(target))
}

suspend fun Robot.s_driveRelative(movement: RobotCentricPosition) {
    s_driveToTarget(currentPosition + movement)
}

suspend fun <T:Robot> T.runAuto(b: PhaseBuilder<T>.() -> Unit) {
    val builder = PhaseBuilder<T>(opMode as RobotOpMode<T>)
    builder.b()
    val phases = builder.build()
    val auto = SequentialPhase("name", phases)
    with(auto) {
        initPhase()
        val timer = ElapsedTime()
        while (loopPhase(timer)) {
            opMode.loop.nextTick()
        }
    }
}