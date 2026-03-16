package org.beargineers.platform.rr

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.beargineers.platform.PinpointConfig
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import java.util.Objects

@Config
class PinpointLocalizer(hardwareMap: HardwareMap, initialPose: Pose2d) : Localizer {
    val driver: GoBildaPinpointDriver
    val initialParDirection: GoBildaPinpointDriver.EncoderDirection
    val initialPerpDirection: GoBildaPinpointDriver.EncoderDirection

    private var txWorldPinpoint: Pose2d
    private var txPinpointRobot = Pose2d(0.0, 0.0, 0.0)

    init {
        driver =
            hardwareMap.get(GoBildaPinpointDriver::class.java, "pinpoint")

        driver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        driver.setOffsets(PinpointConfig.pinpoint_xOffset, PinpointConfig.pinpoint_yOffset, DistanceUnit.CM)

        fun DcMotorSimple.Direction.direction(): GoBildaPinpointDriver.EncoderDirection {
            return if (this == DcMotorSimple.Direction.FORWARD) GoBildaPinpointDriver.EncoderDirection.FORWARD else GoBildaPinpointDriver.EncoderDirection.REVERSED
        }

        initialParDirection = PinpointConfig.pinpoint_xEncoderDirection.direction()
        initialPerpDirection = PinpointConfig.pinpoint_yEncoderDirection.direction()

        driver.setEncoderDirections(initialParDirection, initialPerpDirection)

        driver.resetPosAndIMU()

        txWorldPinpoint = initialPose
    }

    override fun setPose(pose: Pose2d) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse())
    }

    override fun getPose(): Pose2d {
        return txWorldPinpoint.times(txPinpointRobot)
    }

    override fun update(): PoseVelocity2d {
        driver.update()
        if (Objects.requireNonNull<DeviceStatus?>(driver.getDeviceStatus()) == DeviceStatus.READY) {
            txPinpointRobot = Pose2d(
                driver.getPosX(DistanceUnit.INCH),
                driver.getPosY(DistanceUnit.INCH),
                driver.getHeading(UnnormalizedAngleUnit.RADIANS)
            )
            val worldVelocity =
                Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH))
            val robotVelocity =
                Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity)

            return PoseVelocity2d(
                robotVelocity,
                driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
            )
        }
        return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
    }
}
