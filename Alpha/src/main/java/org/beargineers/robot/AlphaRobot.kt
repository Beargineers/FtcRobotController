package org.beargineers.robot

import org.beargineers.R
import org.beargineers.platform.Alliance
import org.beargineers.platform.AprilTagWebcam
import org.beargineers.platform.BLUE_GOAL
import org.beargineers.platform.BLUE_PARK
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Location
import org.beargineers.platform.MecanumDrive
import org.beargineers.platform.RED_GOAL
import org.beargineers.platform.RED_PARK
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.IntakeMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import kotlin.math.abs
import kotlin.math.hypot

@Suppress("unused")
class AlphaRobot(opMode: RobotOpMode<DecodeRobot>) : BaseRobot(opMode), DecodeRobot {
    override val configResource: Int = R.raw.config
    override val drive = MecanumDrive(this)

    override val relativeLocalizer get() = drive.localizerByMotorEncoders
    override val absoluteLocalizer get() = aprilTags

    val aprilTags = AprilTagWebcam(this)
    val shooter = Shooter(this)
    val intake = Intake(this)

    var goalDistanceCM: Double? = null

    val parkCoords: Location =
        (if (opMode.alliance == Alliance.BLUE) BLUE_PARK else RED_PARK).toUnit(DistanceUnit.CM)

    override fun loop() {
        super.loop()

        val goalCoords =
            (if (opMode.alliance == Alliance.BLUE) BLUE_GOAL else RED_GOAL).toUnit(DistanceUnit.CM)

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
        val heading = currentPosition.toAngleUnit(AngleUnit.DEGREES).heading
        val squareAngles = listOf(-180.0, -90.0, 0.0, 90.0, 180.0)
        val parkHeading = squareAngles.minBy { abs(it - heading) }
        driveToTarget(parkCoords.withHeading(parkHeading), 1.0)
    }
}