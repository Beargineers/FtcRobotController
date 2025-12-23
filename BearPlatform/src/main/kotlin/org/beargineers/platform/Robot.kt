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
     * @param positionTolerance Distance threshold in the pose's distance units (default 2.0)
     * @param headingTolerance Angle threshold in the pose's angle units (default 5.0)
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
    fun driveToTarget(target: Position, maxSpeed: Double): Boolean

    /**
     * Follows a spline path to reach a target position with intermediate waypoints.
     *
     * This implements sophisticated path following using:
     * - Catmull-Rom splines for smooth, speed-optimized curves
     * - Pure pursuit algorithm for path tracking
     * - Velocity profiling with acceleration/deceleration
     * - Curvature-based speed adjustment
     *
     * Waypoints serve as guide points rather than strict requirements, allowing
     * the algorithm to optimize the path for maximum speed while maintaining smoothness.
     *
     * @param target Target position (location + heading) to reach
     * @param waypoints List of intermediate waypoints (treated as guidance, not strict points)
     * @param maxSpeed Maximum speed (0.0 to 1.0, as fraction of robot max speed)
     * @return false if target reached, true if still following path
     *
     * ## Example Usage
     * ```kotlin
     * val waypoints = listOf(
     *     Location(50.cm, 20.cm).withHeading(0.degrees),
     *     Location(80.cm, 60.cm).withHeading(45.degrees)
     * )
     * val target = Location(100.cm, 100.cm).withHeading(90.degrees)
     *
     * override fun loop() {
     *     super.loop()
     *     if (followSplinePath(target, waypoints, maxSpeed = 0.8)) {
     *         telemetry.addLine("Following path...")
     *     } else {
     *         telemetry.addLine("Target reached!")
     *         stopDriving()
     *     }
     * }
     * ```
     */
    fun followSplinePath(
        target: Position,
        waypoints: List<Position> = emptyList(),
        maxSpeed: Double = 1.0
    ): Boolean

    /**
     * Stops following the current path
     */
    fun stopFollowingPath()

    fun drive(forwardPower: Double, rightPower: Double, turnPower: Double, slow: Boolean = false)

    fun stopDriving()

    fun isMoving(): Boolean

    fun assumePosition(position: Position)

    fun stop()

    fun init()
    fun loop()

    val telemetry: Telemetry

    val currentPosition: Position

    fun configValue(name: String): String?

    val opMode: RobotOpMode<*>
}