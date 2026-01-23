package org.beargineers.platform

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit

private val executor = Executors.newSingleThreadScheduledExecutor()

class LimelightCam(robot: BaseRobot): Camera(robot) {
    val Camera_positionTolerance by config(1.0)
    val Camera_angleRange by config(15)
    val Camera_headingTolerance by config(1.0)

    private val normalizer: PositionNormalDistribution = PositionNormalDistribution(5)

    private var lastUpdated = 0.0
    private val limelight by hardware<Limelight3A>("limelight")

    @Volatile
    private lateinit var pollingTask: ScheduledFuture<*>

    override fun init() {
        limelight.pipelineSwitch(0)
        limelight.start()

        pollingTask = executor.scheduleWithFixedDelay({
            val latestResult = limelight.latestResult
            if (latestResult.timestamp != lastUpdated && latestResult.isValid && latestResult.staleness < 50) {
                lastUpdated = latestResult.timestamp
                val position = position(latestResult)
                if (position != null) normalizer.update(position)
            }

        }, 0L, 5, TimeUnit.MILLISECONDS)
    }

    override fun stop() {
        limelight.stop()
        pollingTask.cancel(false)
    }

    override fun getRobotPose(): Position? {
        val shift = robot.currentVelocity
        val velocity = hypot(shift.right, shift.forward)
        if (velocity > Camera_positionTolerance.cm) {
            normalizer.reset()
            return null
        }

        val result = normalizer.result(Camera_positionTolerance.cm, Camera_headingTolerance.degrees)
        return result?.takeIf { isGoodResult(it) }
    }

    private fun isGoodResult(result: Position): Boolean {
        return isGoodResultFor(BLUE_GOAL, result) || isGoodResultFor(RED_GOAL, result)
    }

    private fun isGoodResultFor(result: Position, goal: Position): Boolean {
        val headingToGoal = headingFromTo(result.location(), goal.location())
        if (abs(headingToGoal - result.heading) > Camera_angleRange.degrees) return false
        if (abs(goal.heading - result.heading) > Camera_angleRange.degrees) return false
        return true
    }

    fun headingFromTo(from: Location, to: Location): Angle {
        return atan2(from.y - to.y, from.x - to.x)
    }

    private fun position(latestResult: LLResult): Position? {
        return latestResult.botpose?.robotPose()
    }

    private val BLUE_GOAL = Position(-58.3727.inch, -55.6425.inch, -128.degrees)
    private val RED_GOAL = Position(-58.3727.inch, 55.6425.inch, 128.degrees)
}