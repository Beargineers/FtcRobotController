package org.beargineers.platform

import org.firstinspires.ftc.robotcore.external.Telemetry

interface Robot {
    fun <T:Any > getPart(part: Part<T>): T

    fun driveToTarget(target: Position): Boolean

    fun drive(forwardPower: Double, rightPower: Double, turnPower: Double)
    fun driveByPowerAndAngle(theta: Double, power: Double, turn: Double)

    fun stopDriving()

    fun isMoving(): Boolean

    fun assumePosition(position: Position)

    fun stop()

    fun init()
    fun loop()

    fun lowFpsMode(mode: Boolean) {}

    val telemetry: Telemetry

    val currentPosition: Position

    val currentVelocity: RelativePosition

    var targetSpeed: Double

    val opMode: RobotOpMode<*>

    val alliance: Alliance get() = opMode.alliance
}

class RobotLocations(val robot: Robot) {
    companion object : Part<RobotLocations> {
        override fun build(robot: Robot) = RobotLocations(robot)
    }

    val lf_wheel: Location get() = Location(-RobotDimensions.ROBOT_WHEELBASE_WIDTH/2.cm, RobotDimensions.ROBOT_WHEELBASE_LENGTH/2.cm).toAbsolute(robot.currentPosition)
    val rf_wheel: Location get() = Location(RobotDimensions.ROBOT_WHEELBASE_WIDTH/2.cm, RobotDimensions.ROBOT_WHEELBASE_LENGTH/2.cm).toAbsolute(robot.currentPosition)
    val lb_wheel: Location get() = Location(-RobotDimensions.ROBOT_WHEELBASE_WIDTH/2.cm, -RobotDimensions.ROBOT_WHEELBASE_LENGTH/2.cm).toAbsolute(robot.currentPosition)
    val rb_wheel: Location get() = Location(RobotDimensions.ROBOT_WHEELBASE_WIDTH/2.cm, -RobotDimensions.ROBOT_WHEELBASE_LENGTH/2.cm).toAbsolute(robot.currentPosition)

    val lf_corner: Location get() = Location((RobotDimensions.ROBOT_WIDTH / 2).cm, RobotDimensions.ROBOT_FRONT_OFFSET.cm).toAbsolute(robot.currentPosition)
    val rf_corner: Location get() = Location(-(RobotDimensions.ROBOT_WIDTH / 2).cm, RobotDimensions.ROBOT_FRONT_OFFSET.cm).toAbsolute(robot.currentPosition)
    val lb_corner: Location get() = Location((RobotDimensions.ROBOT_WIDTH / 2).cm, -RobotDimensions.ROBOT_BACK_OFFSET.cm).toAbsolute(robot.currentPosition)
    val rb_corner: Location get() = Location(-(RobotDimensions.ROBOT_WIDTH / 2).cm, -RobotDimensions.ROBOT_BACK_OFFSET.cm).toAbsolute(robot.currentPosition)
}

object RobotDimensions {
    val ROBOT_WEIGHT by config(5.0)
    val ROBOT_WHEELBASE_WIDTH by config(0.0)
    val ROBOT_WHEELBASE_LENGTH by config(0.0)

    val ROBOT_LENGTH by config(0.0)
    val ROBOT_WIDTH by config(0.0)
    val ROBOT_FRONT_OFFSET by config(0.0)
    val ROBOT_BACK_OFFSET by config(0.0)
}
