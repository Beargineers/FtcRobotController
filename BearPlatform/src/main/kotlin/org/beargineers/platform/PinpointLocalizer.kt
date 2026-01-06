package org.beargineers.platform

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit

class PinpointLocalizer(robot: BaseRobot): Hardware(robot), RelativeLocalizer {
    private val pinpoint by hardware<GoBildaPinpointDriver>()

    private val pinpoint_xOffset by robot.config(8.1)
    private val pinpoint_yOffset by robot.config(6.0)

    override fun init() {
        pinpoint.setOffsets(
            pinpoint_xOffset,
            pinpoint_yOffset,
            DistanceUnit.CM
        )

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.REVERSED
        )
        pinpoint.resetPosAndIMU()
    }

    override fun loop() {
        pinpoint.update()
    }

    override fun updatePositionEstimate(position: Position) {
        with(pinpoint) {
            setPosX(position.x.cm(), DistanceUnit.CM)
            setPosY(position.y.cm(), DistanceUnit.CM)
            setHeading(position.heading.radians(), AngleUnit.RADIANS)
        }
    }

    override fun getPosition(): Position {
        with(pinpoint) {
            return Position(
                x = getPosX(DistanceUnit.CM).cm,
                y = getPosY(DistanceUnit.CM).cm,
                heading = getHeading(AngleUnit.DEGREES).degrees
            )
        }
    }

    override fun getVelocity(): RelativePosition {
        return RelativePosition(
            pinpoint.getVelX(DistanceUnit.CM).cm,
            pinpoint.getVelY(DistanceUnit.CM).cm,
            pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES).degrees,
        )
    }
}