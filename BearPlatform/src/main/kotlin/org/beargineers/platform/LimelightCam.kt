package org.beargineers.platform

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit

private val executor = Executors.newSingleThreadScheduledExecutor()

class LimelightCam(robot: BaseRobot): Camera(robot) {
    private val normalizer: PositionNormalDistribution = PositionNormalDistribution(1.cm, 1.degrees, 5)

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
        if (robot.isMoving()) {
            normalizer.reset()
            return null
        }

        return normalizer.result()
    }

    private fun position(latestResult: LLResult): Position? {
        val position = latestResult.botpose?.robotPose()?.rotate(CameraPosition_yaw.degrees) ?: return null
        val cameraOffset = Location(CameraPosition_right.cm, CameraPosition_forward.cm)

        return cameraOffset.toAbsolute(position).withHeading(position.heading)
    }
}