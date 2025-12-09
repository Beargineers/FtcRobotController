package org.beargineers.platform

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt


class LimelightCam(robot: BaseRobot): Hardware(robot), AbsoluteLocalizer {
    private val limelight by hardware<Limelight3A>("limelight")

    override fun init() {
        limelight.pipelineSwitch(0)
        limelight.start()
    }

    override fun stop() {
        limelight.stop()
    }

    override fun getRobotPose(): AbsolutePose? {
        val latestResult = limelight.latestResult

        if (!latestResult.isValid || latestResult.staleness > 50) return null

        val confidence = latestResult.fiducialResults.maxOfOrNull { computeTagConfidence(latestResult, it) } ?: 0.0

        return latestResult.botpose?.let {
            AbsolutePose(it.robotPose(), confidence, latestResult.controlHubTimeStampNanos)
        }
    }

    private fun computeTagConfidence(result: LLResult, fid: FiducialResult): Double {
        if (!result.isValid()) return 0.0

        val staleness = result.getStaleness()
        if (staleness > 150) return 0.1 // old data


        // Distance from camera to tag:
        val tagPoseCam = fid.getTargetPoseCameraSpace()
        val x = tagPoseCam.getPosition().x
        val y = tagPoseCam.getPosition().y
        val z = tagPoseCam.getPosition().z
        val dist = sqrt(x * x + y * y + z * z)

        // Map distance to [0,1] (0 m–3 m good, >4 m bad)
        val distScore = 1.0 - min(max((dist - 0.5) / (4.0 - 0.5), 0.0), 1.0)

        // Angle off-center (tx)
        val tx = fid.getTargetXDegrees()
        val angleScore = 1.0 - min(abs(tx) / 30.0, 1.0) // >30° off-center = low confidence

        // Combine (you can tune these weights)
        return 0.5 * distScore + 0.5 * angleScore
    }

}