package org.beargineers.platform

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit

object PinpointConfig {
    val pinpoint_xOffset by config(8.1)
    val pinpoint_yOffset by config(6.0)

    val pinpoint_xEncoderDirection by config(DcMotorSimple.Direction.FORWARD)
    val pinpoint_yEncoderDirection by config(DcMotorSimple.Direction.FORWARD)

    val pinpoint_yawScalar by config(0.0)

    val pinpoint_forwardErrorCorrection by config(0.0)
    val pinpoint_rightErrorCorrection by config(0.0)
}

class PinpointLocalizer(robot: BaseRobot): Hardware(robot), RelativeLocalizer {
    private val pinpoint by hardware<GoBildaPinpointDriver>()
    private var correction = Position.zero()


    override fun init() {
        pinpoint.setOffsets(
            PinpointConfig.pinpoint_xOffset,
            PinpointConfig.pinpoint_yOffset,
            DistanceUnit.CM
        )

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        pinpoint.setEncoderDirections(
            PinpointConfig.pinpoint_xEncoderDirection.direction(),
            PinpointConfig.pinpoint_yEncoderDirection.direction()
        )
        pinpoint.resetPosAndIMU()

        val yawScalar = PinpointConfig.pinpoint_yawScalar
        if (yawScalar > 0.0001) {
            pinpoint.setYawScalar(yawScalar)
        }
    }

    private fun DcMotorSimple.Direction.direction(): GoBildaPinpointDriver.EncoderDirection {
        return if (this == DcMotorSimple.Direction.FORWARD) GoBildaPinpointDriver.EncoderDirection.FORWARD else GoBildaPinpointDriver.EncoderDirection.REVERSED
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

    override fun getPosition(oldPosition: Position): Position {
        val newEstimate =with(pinpoint) {
             Position(
                x = getPosX(DistanceUnit.CM).cm,
                y = getPosY(DistanceUnit.CM).cm,
                heading = getHeading(AngleUnit.DEGREES).degrees
            )
        }

        /*
        correction += errorCorrection(oldPosition, newEstimate)

        telemetry.addData("Correction", correction)
        */

        return newEstimate
    }

    fun errorCorrection(oldPos: Position, newPos: Position): RobotCentricPosition {
        val (deltaX, deltaY, deltaH) = newPos - oldPos

        val N = 10
        val dx = deltaX / N.toDouble()
        val dy = deltaY / N.toDouble()
        val dh = deltaH / N.toDouble()

        val magnitude = hypot(dx, dy)
        val absoluteH = atan2(dy,dx)

        var deltaForward = 0.cm
        var deltaRight = 0.cm
        var heading = oldPos.heading

        repeat(N) {
            val relHeading = absoluteH - heading
            heading += dh
            deltaForward += magnitude * cos(relHeading)
            deltaRight += magnitude * sin(relHeading)
        }

        return RobotCentricPosition(
            deltaForward * PinpointConfig.pinpoint_forwardErrorCorrection,
            deltaRight * PinpointConfig.pinpoint_rightErrorCorrection,
            0.degrees
        )
    }

    override fun getVelocity(): RobotCentricPosition {
        return RobotCentricPosition(
            pinpoint.getVelX(DistanceUnit.CM).cm,
            pinpoint.getVelY(DistanceUnit.CM).cm,
            pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES).degrees,
        )
    }
}