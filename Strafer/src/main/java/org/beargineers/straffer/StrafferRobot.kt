package org.beargineers.straffer

import com.qualcomm.robotcore.hardware.DistanceSensor
import org.beargineers.platform.AbsoluteLocalizer
import org.beargineers.platform.Angle
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Hardware
import org.beargineers.platform.LimelightCam
import org.beargineers.platform.MecanumDrive
import org.beargineers.platform.PinpointLocalizer
import org.beargineers.platform.RelativeLocalizer
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.degrees
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class Sensors(robot: BaseRobot) : Hardware(robot) {
    val distance by hardware<DistanceSensor>()

    override fun loop() {
        super.loop()
        telemetry.addData("DISTANCE", distance.getDistance(DistanceUnit.CM))
    }
}

class StrafferRobot(op: RobotOpMode<DecodeRobot>) : BaseRobot(op), DecodeRobot {
    override val drive = MecanumDrive(this)

    override val absoluteLocalizer: AbsoluteLocalizer = LimelightCam(this)
    override val relativeLocalizer: RelativeLocalizer = PinpointLocalizer(this)

    val sensors = Sensors(this)

    override val configResource: Int = R.raw.config
    override val intakeMode: IntakeMode get() = IntakeMode.OFF
    override val shootingAngleCorrection: Angle
        get() = 0.degrees

    override fun intakeMode(mode: IntakeMode) {
        // TODO("Not yet implemented")
    }

    override fun launch() {
        // TODO("Not yet implemented")
    }

    override fun enableFlywheel(on: Boolean) {
        // TODO("Not yet implemented")
    }

    override fun isShooting(): Boolean {
        return false
    }
}