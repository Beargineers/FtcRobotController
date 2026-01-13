@file:Suppress("unused")

package org.beargineers.platform

import kotlin.math.abs

/**
 * Manages the state and execution of following a specific Path.
 *
 * A PathFollower is created when you start following a new Path instance.
 * It holds all the state required for following (progress, speed, generated spline, etc.).
 *
 * Each PathFollower is bound to a specific Path and starting position.
 *
 * @param robot The robot instance to control
 * @param path The immutable Path definition to follow
 * @param startPosition Where the robot starts following this path
 */
internal class PathFollower(
    private val robot: BaseRobot,
    val path: List<Position>,
    startPosition: Position
) {
    private var lastTargetIndex: Int = 0
    private var currentSpeed: Double = 0.0
    private var lastUpdateNanos: Long = System.nanoTime()
    private val distancePID = PID(
        integralZone = 10.0,
        integralMax = 10000.0,
        outputMin = -1.0, outputMax = 1.0
    )
    private val headingPID = PID(
        integralZone = 10.0,
        integralMax = 10000.0,
        outputMin = -1.0, outputMax = 1.0
    )

    init {
        distancePID.updateCoefficients(robot.position_P, robot.position_I, robot.position_D)
        headingPID.updateCoefficients(robot.heading_P, robot.heading_I, robot.heading_D)
    }

    /**
     * Execute one update cycle of path following.
     * Calculates drive powers and commands the robot directly.
     *
     * @return true if still following, false if target reached
     */
    fun update(): Boolean {
        if (lastTargetIndex > path.lastIndex) return false
        val currentPosition = robot.currentPosition

        val headingToTarget = atan2(
            path[lastTargetIndex].y - robot.currentPosition.y,
            path[lastTargetIndex].x - robot.currentPosition.x)

        if (lastTargetIndex < path.lastIndex &&  currentPosition.distanceTo(path[lastTargetIndex]) < 8.cm) {

             lastTargetIndex++
        }

        val currentNanos = System.nanoTime()
        lastUpdateNanos = currentNanos

        val currentTarget = path[lastTargetIndex]

        val positionError = currentPosition.distanceTo(currentTarget).cm()
        val headingError = (currentTarget.heading - currentPosition.heading).normalize().degrees()

        distancePID.updateCoefficients(robot.position_P, robot.position_I, robot.position_D)
        headingPID.updateCoefficients(robot.heading_P, robot.heading_I, robot.heading_D)

        distancePID.updateError(positionError)
        headingPID.updateError(headingError)

        robot.telemetry.addData("Position E", positionError)
        robot.telemetry.addData("Heading E", abs(headingError))

        val finished = lastTargetIndex == path.lastIndex &&
                currentPosition.distanceTo(path.last()).cm() < robot.positionTolerance &&
                abs(currentPosition.heading - path.last().heading).degrees() < robot.headingTolerance

        if (finished) {
            robot.stopDriving()
            return false
        }

        fun Double.dezeroify(): Double {
            val min = robot.minimalWheelPower
            return if (abs(this) > 0.0001 && abs(this) < min) min else this
        }

        val movePower = distancePID.result().coerceIn(-1.0, 1.0).dezeroify()
        val turnPower = -headingPID.result().coerceIn(-1.0, 1.0).dezeroify()

//        headingPID.logOscillation(robot.telemetry)

        // Apply drive power to robot
        robot.driveByPowerAndAngle((headingToTarget - currentPosition.heading).normalize().radians(), movePower, turnPower)

        return true
    }
}