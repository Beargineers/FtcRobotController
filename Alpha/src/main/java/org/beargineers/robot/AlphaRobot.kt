package org.beargineers.robot

import org.beargineers.R
import org.beargineers.platform.Alliance
import org.beargineers.platform.AprilTagWebcam
import org.beargineers.platform.BLUE_PARK
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Location
import org.beargineers.platform.MecanumDrive
import org.beargineers.platform.RED_PARK
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.abs
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.degrees
import org.beargineers.platform.goalDistance
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection

@Suppress("unused")
class AlphaRobot(opMode: RobotOpMode<DecodeRobot>) : BaseRobot(opMode), DecodeRobot {
    override val configResource: Int = R.raw.config
    override val drive = MecanumDrive(this)

    override val relativeLocalizer get() = drive.localizerByMotorEncoders
    override val absoluteLocalizer get() = aprilTags

    val aprilTags = AprilTagWebcam(this)
    val shooter = Shooter(this)
    val intake = Intake(this)
    val parkCoords: Location =
        (if (opMode.alliance == Alliance.BLUE) BLUE_PARK else RED_PARK)

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
        intake.enable(mode)
    }

    override fun launch() {
        shooter.launch()
    }

    override fun enableFlywheel(on: Boolean) {
        shooter.enableFlywheel(on)
    }

    override fun isShooting(): Boolean {
        return shooter.feederStartedAt != 0L
    }

    override fun park(){
        val heading = currentPosition.heading
        val squareAngles = listOf(-180.degrees, -90.degrees, 0.degrees, 90.degrees, 180.degrees)
        val parkHeading = squareAngles.minBy { abs(it - heading) }
        driveToTarget(parkCoords.withHeading(parkHeading), 1.0)
    }
}