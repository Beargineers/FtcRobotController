package org.beargineers.platform

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit

private val executor = Executors.newSingleThreadScheduledExecutor()

class LimelightCam(robot: BaseRobot, val camHeading: () -> Angle): Camera(robot) {
    val Camera_positionTolerance by config(1.0)
    val Camera_angleRange by config(15)
    val Camera_headingTolerance by config(1.0)

    private val normalizerMT2: PositionMedian = PositionMedian(5)
    private val normalizer: PositionMedian = PositionMedian(5)

    private var lastUpdated = 0.0
    private val limelight by hardware<Limelight3A>("limelight")

    private var yaw: Double = 0.0

    @Volatile
    private lateinit var pollingTask: ScheduledFuture<*>

    override fun init() {
        limelight.pipelineSwitch(0)
        limelight.start()

        pollingTask = executor.scheduleWithFixedDelay({
            limelight.updateRobotOrientation(yaw)

            val latestResult = limelight.latestResult
            if (latestResult.timestamp != lastUpdated && latestResult.isValid && latestResult.staleness < 50) {
                lastUpdated = latestResult.timestamp

                positionMT2(latestResult)?.let { normalizerMT2.update(it) }
                position(latestResult)?.let { normalizer.update(it) }
            }

        }, 0L, 5, TimeUnit.MILLISECONDS)
    }

    override fun stop() {
        limelight.stop()
        pollingTask.cancel(false)
    }

    override fun loop() {
        yaw = camHeading().degrees()
/*
        val result = normalizer.result(
            Camera_positionTolerance.cm,
            Camera_headingTolerance.degrees
        )
        if (result != null) {
            Frame.graph("TURRET HEADING DRIFT", abs((result.heading - camHeading()).normalize().degrees()))
        }
        */

    }

    override fun getRobotPose(): Position? {
        val shift = robot.currentVelocity
        val velocity = hypot(shift.x, shift.y)
        if (velocity > Camera_positionTolerance.cm) {
            normalizer.reset()
            normalizerMT2.reset()
            return null
        }

        val result = normalizerMT2.result(
            Camera_positionTolerance.cm,
            Camera_headingTolerance.degrees,
        )
        return result?.takeIf { isGoodResult(it) }
    }

    private fun isGoodResult(result: Position): Boolean {
        return isGoodResultFor(BLUE_GOAL, result) || isGoodResultFor(RED_GOAL, result)
    }

    private fun isGoodResultFor(goal: Position, result: Position): Boolean {
        val headingToGoal = headingFromTo(result.location(), goal.location())
        if (abs((headingToGoal - result.heading).normalize()) > Camera_angleRange.degrees) return false
        if (abs((goal.heading - result.heading).normalize()) > Camera_angleRange.degrees) return false
        return true
    }

    private fun positionMT2(latestResult: LLResult): Position? {
        return latestResult.botpose_MT2?.robotPose()?.convertFromCameraPosition()
    }

    private fun position(latestResult: LLResult): Position? {
        return latestResult.botpose?.robotPose()?.convertFromCameraPosition()
    }

    private fun Position.convertFromCameraPosition(): Position {
        return RobotCentricLocation(-CameraPosition_forward.cm, -CameraPosition_right.cm).toFieldCentric(this).withHeading(heading)
    }

    private val BLUE_GOAL = Position(-58.3727.inch, -55.6425.inch, -128.degrees)
    private val RED_GOAL = Position(-58.3727.inch, 55.6425.inch, 128.degrees)
}
