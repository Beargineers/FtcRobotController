package org.beargineers.platform

import com.qualcomm.hardware.limelightvision.Limelight3A


class LimelightCam(robot: BaseRobot): Hardware(robot), AbsoluteLocalizer {
    private val limelight by hardware<Limelight3A>("limelight")

    override fun init() {
        limelight.pipelineSwitch(0)
        limelight.start()
    }

    override fun stop() {
        limelight.stop()
    }

    override fun getRobotPose(): Position? {
        val latestResult = limelight.latestResult

        if (!latestResult.isValid || latestResult.staleness > 50) return null

        return latestResult.botpose?.robotPose()
    }
}