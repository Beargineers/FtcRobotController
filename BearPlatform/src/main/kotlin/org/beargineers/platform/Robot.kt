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
}