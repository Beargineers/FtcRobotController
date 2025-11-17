package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.internal.Hardware
import org.firstinspires.ftc.teamcode.internal.RobotOpModeBase
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

abstract class Robot() : RobotOpModeBase() {
    lateinit var drive: Drivebase
    lateinit var aprilTags: AprilTagWebcam
    lateinit var shooter: Shooter
    lateinit var intake: Intake

    val allHardware = mutableListOf<Hardware>()

    var currentPose: Pose2D? = null

    override fun init() {
        drive = Drivebase(this)
        aprilTags = AprilTagWebcam(this)
        shooter = Shooter(this)
        intake = Intake(this)

        allHardware += drive
        allHardware += aprilTags
        allHardware += shooter
        allHardware += intake

        allHardware.forEach {
            it.init()
        }

        telemetry.addLine("Ready")
    }

    override fun loop() {
        allHardware.forEach { it.loop() }
        currentPose = aprilTags.robotPose()
        telemetry.addData("Pose", currentPose)
    }

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
     * @param pose Target pose (position and heading) to drive to
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
    fun driveToPose(
        pose: Pose2D,
        maxSpeed: Double = 0.5,
        positionTolerance: Double = 2.0,
        headingTolerance: Double = 5.0
    ): Boolean {
        if (currentPose == null) {
            telemetry.addLine("Current position is unknown, cannot drive to position")
            return false
        }

        // Make sure current and target positions are in the same units. Also, angle units have to be in radians for calling sin/cos etc.
        val cp = currentPose!!.toDistanceUnit(DistanceUnit.INCH).toAngleUnit(AngleUnit.RADIANS)
        val tp = pose.toDistanceUnit(DistanceUnit.INCH).toAngleUnit(AngleUnit.RADIANS)

        // Calculate position error (field frame)
        val deltaX = tp.x - cp.x
        val deltaY = tp.y - cp.y

        // Calculate distance to target
        val distanceToTarget = sqrt(deltaX.pow(2) + deltaY.pow(2))

        // Calculate heading error (normalized to -180 to 180 degrees equivalent)
        var headingError = tp.heading - cp.heading
        val halfCircle = PI
        val fullCircle = halfCircle * 2
        // Normalize heading error to [-180, 180] or [-π, π]
        while (headingError > halfCircle) headingError -= fullCircle
        while (headingError < -halfCircle) headingError += fullCircle

        // Check if we've reached the target
        if (distanceToTarget < positionTolerance && abs(headingError) < headingTolerance) {
            drive.stop()
            return true
        }

        // Convert field-frame error to robot-frame error
        // We need to rotate the field-frame delta by the negative of the robot's heading
        val robotHeadingRad = cp.heading
        val robotFrameX = deltaX * cos(robotHeadingRad) + deltaY * sin(robotHeadingRad)
        val robotFrameY = -deltaX * sin(robotHeadingRad) + deltaY * cos(robotHeadingRad)

        // Proportional control gains (tune these values for your robot)
        val kP_position = 0.02  // Position gain (power per distance unit)
        val kP_heading = 0.01   // Heading gain (power per angle unit)

        // Calculate drive powers using proportional control
        val forwardPower = robotFrameY * kP_position
        val strafePower = robotFrameX * kP_position
        val turnPower = headingError * kP_heading

        fun clamp(value: Double): Double {
            return value.coerceIn(-maxSpeed, maxSpeed)
        }

        // Apply drive power
        drive.drive(
            y = clamp(forwardPower),
            x = clamp(strafePower),
            turn = clamp(turnPower)
        )

        // Add telemetry for debugging
        telemetry.addData("Distance to target", "%.2f %s".format(distanceToTarget, tp.distanceUnit))
        telemetry.addData("Heading error", "%.2f %s".format(headingError, tp.angleUnit))
        telemetry.addData("Drive powers (y,x,turn)", "%.2f, %.2f, %.2f".format(
            clamp(forwardPower), clamp(strafePower), clamp(turnPower)
        ))

        return false
    }

    override fun stop() {
        allHardware.forEach { it.stop() }
    }
}