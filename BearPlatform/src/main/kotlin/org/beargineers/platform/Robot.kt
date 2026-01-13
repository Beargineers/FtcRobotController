package org.beargineers.platform

import org.firstinspires.ftc.robotcore.external.Telemetry

interface Robot {
    /**
     * Continuously drives the robot toward a target pose using proportional control.
     *
     * This function calculates the error between the current pose and target pose,
     * then applies proportional power to the mecanum drive motors to reduce that error.
     * This function should be called repeatedly in a loop until the robot reaches the target.
     *
     * The function uses simple proportional (P) control:
     * - Position error is converted to drive power (forward/strafe)
     * - Heading error is converted to rotational power
     * - Power is clamped to safe limits
     *
     * @param target Target pose (position and heading) to drive to
     * @return true if robot has reached the target within tolerances, false otherwise
     *
     * ## Example Usage
     * ```kotlin
     * val targetPose = Pose2D(100.0, 50.0, 90.0, DistanceUnit.CM, AngleUnit.DEGREES)
     *
     * override fun loop() {
     *     super.loop()
     *     if (!driveToPose(targetPose)) {
     *         telemetry.addLine("Driving to target...")
     *     } else {
     *         telemetry.addLine("Target reached!")
     *         drive.stop()
     *     }
     * }
     * ```
     */
    fun driveToTarget(target: Position): Boolean

    fun followPath(path: List<Position>): Boolean

    /**
     * Stops following the current path
     */
    fun stopFollowingPath()

    fun drive(forwardPower: Double, rightPower: Double, turnPower: Double)
    fun driveByPowerAndAngle(theta: Double, power: Double, turn: Double)

    fun stopDriving()

    fun isMoving(): Boolean

    fun assumePosition(position: Position)

    fun stop()

    fun init()
    fun loop()

    val telemetry: Telemetry

    val currentPosition: Position

    val currentVelocity: RelativePosition

    var targetSpeed: Double

    fun configValue(name: String): String?

    val opMode: RobotOpMode<*>

    val dimensions: RobotDimensions

    val lf_wheel: Location get() = Location(-dimensions.ROBOT_WHEELBASE_WIDTH/2.cm, dimensions.ROBOT_WHEELBASE_LENGTH/2.cm).toAbsolute(currentPosition)
    val rf_wheel: Location get() = Location(dimensions.ROBOT_WHEELBASE_WIDTH/2.cm, dimensions.ROBOT_WHEELBASE_LENGTH/2.cm).toAbsolute(currentPosition)
    val lb_wheel: Location get() = Location(-dimensions.ROBOT_WHEELBASE_WIDTH/2.cm, -dimensions.ROBOT_WHEELBASE_LENGTH/2.cm).toAbsolute(currentPosition)
    val rb_wheel: Location get() = Location(dimensions.ROBOT_WHEELBASE_WIDTH/2.cm, -dimensions.ROBOT_WHEELBASE_LENGTH/2.cm).toAbsolute(currentPosition)

    val lf_corner: Location get() = Location((dimensions.ROBOT_WIDTH / 2).cm, dimensions.ROBOT_FRONT_OFFSET.cm).toAbsolute(currentPosition)
    val rf_corner: Location get() = Location(-(dimensions.ROBOT_WIDTH / 2).cm, dimensions.ROBOT_FRONT_OFFSET.cm).toAbsolute(currentPosition)
    val lb_corner: Location get() = Location((dimensions.ROBOT_WIDTH / 2).cm, -dimensions.ROBOT_BACK_OFFSET.cm).toAbsolute(currentPosition)
    val rb_corner: Location get() = Location(-(dimensions.ROBOT_WIDTH / 2).cm, -dimensions.ROBOT_BACK_OFFSET.cm).toAbsolute(currentPosition)
}

class RobotDimensions(val robot: Robot) {
    val ROBOT_WHEELBASE_WIDTH by robot.config(0.0)
    val ROBOT_WHEELBASE_LENGTH by robot.config(0.0)

    val ROBOT_LENGTH by robot.config(0.0)
    val ROBOT_WIDTH by robot.config(0.0)
    val ROBOT_FRONT_OFFSET by robot.config(0.0)
    val ROBOT_BACK_OFFSET by robot.config(0.0)
}
