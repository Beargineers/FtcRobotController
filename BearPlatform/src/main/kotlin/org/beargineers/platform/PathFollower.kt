@file:Suppress("unused")

package org.beargineers.platform

import com.qualcomm.robotcore.util.ElapsedTime
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
    val path: List<Waypoint>,
    startPosition: Position
) {
    private var lastTargetIndex: Int = 0
    private var currentSpeed: Double = 0.0
    private var lastTimeMoved = ElapsedTime()
    private val drivePID = PID(
        integralZone = 20.0,
        integralMax = 3000.0,
        outputMin = -1.0, outputMax = 1.0
    )
    private val translationalPID = PID(
        integralZone = 20.0,
        integralMax = 3000.0,
        outputMin = -1.0, outputMax = 1.0
    )
    private val headingPID = PID(
        integralZone = 5.0,
        integralMax = 3000.0,
        outputMin = -1.0, outputMax = 1.0
    )

    init {
        drivePID.updateCoefficients(robot.drive_K)
        headingPID.updateCoefficients(robot.heading_K)
        translationalPID.updateCoefficients(robot.heading_K)
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
        var currentWaypoint = path[lastTargetIndex]
        var currentTarget = currentWaypoint.target

        if (lastTargetIndex < path.lastIndex &&  currentPosition.distanceTo(path[lastTargetIndex].target) < 8.cm) {
            currentWaypoint.onArrival()
            lastTargetIndex++
            currentWaypoint = path[lastTargetIndex]
            currentTarget = currentWaypoint.target
        }

        if (robot.isMoving()) {
            lastTimeMoved.reset()
        }
        else if (lastTimeMoved.milliseconds() > robot.stalledPathAbortTimeoutMillis) {
            return false
        }

        val (dForward, dRight) = currentTarget.location().toRobotFrame(robot)

        val driveError = dForward.cm()
        val translationalError = dRight.cm()
        val headingError = (currentTarget.heading - currentPosition.heading).normalize().degrees()

        drivePID.updateCoefficients(robot.drive_K)
        translationalPID.updateCoefficients(robot.translational_K)
        headingPID.updateCoefficients(robot.heading_K)

        drivePID.updateError(driveError)
        translationalPID.updateError(translationalError)
        headingPID.updateError(headingError)

        robot.panelsTelemetry.addData("DriveE", driveError)
        robot.panelsTelemetry.addData("TransE", translationalError)
        robot.panelsTelemetry.addData("HeadE", abs(headingError))

        headingPID.logErrors(robot.panelsTelemetry)
        headingPID.logOscillation(robot.panelsTelemetry)

        val finished = lastTargetIndex == path.lastIndex &&
                currentPosition.distanceTo(path.last().target).cm() < robot.positionTolerance &&
                abs(currentPosition.heading - path.last().target.heading).degrees() < robot.headingTolerance

        if (finished) {
            currentWaypoint.onArrival()
            robot.stopDriving()
            return false
        }

        fun Double.dezeroify(): Double {
            val min = robot.minimalWheelPower
            return if (abs(this) > 0.0001 && abs(this) < min) min else this
        }


//        headingPID.logOscillation(robot.telemetry)

        val forwardPower = drivePID.result().dezeroify()
        val strafePower = translationalPID.result().dezeroify()
        val turnPower = -headingPID.result().dezeroify()

        robot.targetSpeed = currentWaypoint.speed
        robot.drive(forwardPower,strafePower, turnPower)

        return true
    }
}