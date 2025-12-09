package org.beargineers.platform

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit

class PinpointLocalizer(op: BaseRobot): Hardware(op), RelativeLocalizer {
    private val pinpoint by hardware<GoBildaPinpointDriver>()

    override fun init() {
        pinpoint.setOffsets(
            -84.0,
            -168.0,
            DistanceUnit.MM
        ) //these are tuned for 3110-0002-0001 Product Insight #1

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        )
        pinpoint.resetPosAndIMU()
    }

    override fun loop() {
        pinpoint.update()
    }

    override fun updatePositionEstimate(position: Position) {
        with(pinpoint) {
            setPosX(position.x, position.distanceUnit)
            setPosY(position.y, position.distanceUnit)
            setHeading(position.heading, position.angleUnit)
        }
    }

    override fun getPosition(): Position {
        with(pinpoint) {
            return Position(
                x = getPosX(DISTANCE_UNIT),
                y = getPosY(DISTANCE_UNIT),
                heading = getHeading(ANGLE_UNIT),
                distanceUnit = DISTANCE_UNIT,
                angleUnit = ANGLE_UNIT
            )
        }
    }

    override fun getVelocity(): RelativePosition {
        return RelativePosition(
            pinpoint.getVelX(DISTANCE_UNIT),
            pinpoint.getVelY(DISTANCE_UNIT),
            pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES),
            DISTANCE_UNIT, AngleUnit.DEGREES
        )
    }
}