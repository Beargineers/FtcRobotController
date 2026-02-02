package org.beargineers.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.beargineers.platform.Alliance
import org.beargineers.platform.Angle
import org.beargineers.platform.AprilTagWebcam
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.FusionLocalizer
import org.beargineers.platform.Localizer
import org.beargineers.platform.MecanumDrive
import org.beargineers.platform.MecanumEncodersLocalizers
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.decode.goalDistance
import org.beargineers.platform.degrees
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection

@Suppress("unused")
class AlphaRobot(opMode: RobotOpMode<DecodeRobot>) : BaseRobot(opMode), DecodeRobot {
    override val drive = MecanumDrive(this)

    val aprilTags = AprilTagWebcam(this)
    val shooter = Shooter(this)
    val intake = Intake(this)

    override val localizer: Localizer = FusionLocalizer(telemetry, aprilTags, MecanumEncodersLocalizers(this, drive))

    override fun loop() {
        super.loop()

        telemetry.addData("Goal distance", goalDistance())
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

    override fun intakeMode(mode: IntakeMode) {
        intake.mode = mode
    }

    override val intakeMode: IntakeMode get() = intake.mode
    override val shootingAngleCorrection: Angle
        get() = 0.degrees
    override val artifactsCount: Int
        get() = 0

    override fun launch() {
        shooter.launch()
    }

    override fun enableFlywheel(on: Boolean) {
        shooter.enableFlywheel(on)
    }

    override fun isShooting(): Boolean {
        return shooter.feederStartedAt != 0L
    }

    override fun adjustShooting(distance: Double, angle: Double) {

    }

    fun setMotorPower(motor: DcMotor, power: Double, compensate: Boolean = true) {
        val ticksPerSecond = motor.motorType.achieveableMaxTicksPerSecondRounded
        if (motor is DcMotorEx && ticksPerSecond > 0) {
            motor.velocity = power * ticksPerSecond
        }
        else {
            val nominalVoltage = 12.0
            val voltage = if (compensate) opMode.hardwareMap.voltageSensor.iterator().next().voltage else nominalVoltage
            val compensation = voltage / nominalVoltage
            motor.power = (power / compensation).coerceIn(-1.0, 1.0)
        }
    }

    override fun getReadyForShoot() {
        TODO("Not yet implemented")
    }

    override fun warnDriverON() {
    }

}