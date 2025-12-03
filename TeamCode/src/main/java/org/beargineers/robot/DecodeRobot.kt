package org.beargineers.robot

import org.beargineers.platform.Alliance
import org.beargineers.platform.AprilTagWebcam
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.KalmanFilter
import org.beargineers.platform.MecanumDrive
import org.beargineers.platform.RobotOpMode
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection

class DecodeRobot(opMode: RobotOpMode<DecodeRobot>) : BaseRobot(opMode) {
    override val drive = MecanumDrive(this)

    override val relativeLocalizer get() = drive.localizerByMotorEncoders
    override val absoluteLocalizer get() = aprilTags

    override fun configureKalmanFilter(): KalmanFilter {
        return KalmanFilter(
            processNoisePosition = KalmanFilterConfig.PROCESS_NOISE_POSITION,
            processNoiseHeading = KalmanFilterConfig.PROCESS_NOISE_HEADING,
            measurementNoisePosition = KalmanFilterConfig.MEASUREMENT_NOISE_POSITION,
            measurementNoiseHeading = KalmanFilterConfig.MEASUREMENT_NOISE_HEADING
        )
    }

    val aprilTags = AprilTagWebcam(this,
        CameraPosition.forward, CameraPosition.right, CameraPosition.up,
        CameraPosition.pitch, CameraPosition.yaw, CameraPosition.roll)
    val shooter = Shooter(this)
    val intake = Intake(this)

    var goalDistanceCM: Double? = null
    var savedGoalDistanceCM: Double = 90.0

    override fun loop() {
        super.loop()

        if (isMoving()) {
            goalDistanceCM = null
        }

        val target = findTarget()
        if (target != null) {
            goalDistanceCM = target.ftcPose.range
        }

        telemetry.addData("Goal distance", goalDistanceCM ?: "goal not found")
    }

    fun findTarget(): AprilTagDetection? {
        return when (opMode.alliance) {
            Alliance.RED -> findRedTarget()
            Alliance.BLUE -> findBlueTarget()
        }
    }

    fun findRedTarget() : AprilTagDetection? {
        return aprilTags.getAprilReadings(24).singleOrNull()
    }

    fun findBlueTarget() : AprilTagDetection? {
        return aprilTags.getAprilReadings(20).singleOrNull()
    }
}