package org.beargineers.robot

import org.beargineers.platform.Alliance
import org.beargineers.platform.AprilTagWebcam
import org.beargineers.platform.AutonomousDriveConfig
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.KalmanFilter
import org.beargineers.platform.MecanumDrive
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.tileLocation
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import kotlin.math.hypot

class DecodeRobot(opMode: RobotOpMode<DecodeRobot>) : BaseRobot(opMode) {
    override val drive = MecanumDrive(this, WheelCorrections.asConfig())

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

    override fun configureAutonomousDriving(): AutonomousDriveConfig {
        return AutonomousDriveConfig(
            minimalWheelPower = AutonomousConfig.MINIMAL_WHEEL_POWER,
            kP_position = AutonomousConfig.kP_position,
            kP_heading = AutonomousConfig.kP_heading
        )
    }

    val aprilTags = AprilTagWebcam(this,
        CameraPosition.forward, CameraPosition.right, CameraPosition.up,
        CameraPosition.pitch, CameraPosition.yaw, CameraPosition.roll)
    val shooter = Shooter(this)
    val intake = Intake(this)

    var goalDistanceCM: Double? = null

    override fun loop() {
        super.loop()

        val goalCoords =
            (if (opMode.alliance == Alliance.BLUE) tileLocation("A6")
            else tileLocation("F6")).toUnit(DistanceUnit.CM)

        val cp = currentPosition.toDistanceUnit(DistanceUnit.CM)
        goalDistanceCM = hypot(cp.x - goalCoords.x, cp.y - goalCoords.y)

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