package org.beargineers.straffer

import com.qualcomm.robotcore.hardware.DistanceSensor
import org.beargineers.platform.Angle
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.FusionLocalizer
import org.beargineers.platform.Hardware
import org.beargineers.platform.LimelightCam
import org.beargineers.platform.Localizer
import org.beargineers.platform.MecanumDrive
import org.beargineers.platform.PinpointLocalizer
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

    override val localizer: Localizer = FusionLocalizer(telemetry, LimelightCam(this), PinpointLocalizer(this))

    val sensors = Sensors(this)
    override fun adjustShooting(distance: Double, angle: Double) {

    }

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