package org.beargineers.platform

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job

interface Robot {
    fun <T:Any > getPart(part: Part<T>): T

    fun assumePosition(position: Position, turretAngle: Angle)

    fun init()
    fun start()
    fun loop()
    fun stop()

    val currentPosition: Position

    val currentVelocity: Position

    fun predictedPosition(nTicks: Int): Position

    val opMode: RobotOpMode<*>

    val alliance: Alliance get() = opMode.alliance
}

fun Robot.submitJob(block: suspend CoroutineScope.() -> Unit): Job {
    return opMode.submitJob(block)
}

suspend fun Robot.nextTick() {
    opMode.nextTick()
}

class RobotLocations(val robot: Robot) {
    companion object : Part<RobotLocations> {
        override fun build(robot: Robot) = RobotLocations(robot)
    }

    val lf_wheel: Location get() = RobotCentricLocation(RobotDimensions.ROBOT_WHEELBASE_LENGTH/2, -RobotDimensions.ROBOT_WHEELBASE_WIDTH/2).toFieldCentric(robot.currentPosition)
    val rf_wheel: Location get() = RobotCentricLocation(RobotDimensions.ROBOT_WHEELBASE_LENGTH/2, RobotDimensions.ROBOT_WHEELBASE_WIDTH/2).toFieldCentric(robot.currentPosition)
    val lb_wheel: Location get() = RobotCentricLocation(-RobotDimensions.ROBOT_WHEELBASE_LENGTH/2, -RobotDimensions.ROBOT_WHEELBASE_WIDTH/2).toFieldCentric(robot.currentPosition)
    val rb_wheel: Location get() = RobotCentricLocation(-RobotDimensions.ROBOT_WHEELBASE_LENGTH/2, RobotDimensions.ROBOT_WHEELBASE_WIDTH/2).toFieldCentric(robot.currentPosition)

    val lf_corner: Location get() = RobotCentricLocation(RobotDimensions.ROBOT_FRONT_OFFSET, (RobotDimensions.ROBOT_WIDTH / 2)).toFieldCentric(robot.currentPosition)
    val rf_corner: Location get() = RobotCentricLocation(RobotDimensions.ROBOT_FRONT_OFFSET, -(RobotDimensions.ROBOT_WIDTH / 2)).toFieldCentric(robot.currentPosition)
    val lb_corner: Location get() = RobotCentricLocation(-RobotDimensions.ROBOT_BACK_OFFSET, (RobotDimensions.ROBOT_WIDTH / 2)).toFieldCentric(robot.currentPosition)
    val rb_corner: Location get() = RobotCentricLocation(-RobotDimensions.ROBOT_BACK_OFFSET, -(RobotDimensions.ROBOT_WIDTH / 2)).toFieldCentric(robot.currentPosition)
}

object RobotDimensions {
    val ROBOT_WEIGHT by config(5.0)
    val ROBOT_WHEELBASE_WIDTH by config(18.inch)
    val ROBOT_WHEELBASE_LENGTH by config(18.inch)

    val ROBOT_LENGTH by config(18.inch)
    val ROBOT_WIDTH by config(18.inch)
    val ROBOT_FRONT_OFFSET by config(9.inch)
    val ROBOT_BACK_OFFSET by config(9.inch)
}
