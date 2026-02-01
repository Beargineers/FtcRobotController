package org.beargineers.beta

import org.beargineers.platform.Angle
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.FusionLocalizer
import org.beargineers.platform.LimelightCam
import org.beargineers.platform.Localizer
import org.beargineers.platform.MecanumDrive
import org.beargineers.platform.PinpointLocalizer
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.decode.goalDistance
import org.beargineers.platform.decode.headingToGoal
import org.beargineers.platform.degrees
import kotlin.time.Duration.Companion.milliseconds

class BetaRobot(op: RobotOpMode<DecodeRobot>) : BaseRobot(op), DecodeRobot {
    var lowFPSMode = false
    private var manualAngleCorrection = 0.0
    override val drive = MecanumDrive(this)
    override fun adjustShooting(distance: Double, angle: Double) {
        shooter.manualPowerAdjustment += distance
        manualAngleCorrection += angle
    }

    override val localizer: Localizer =
        FusionLocalizer(telemetry, LimelightCam(this), PinpointLocalizer(this))

    val shooter = Shooter(this)

    val LedIndicator = LedIndicator(this)
    val intake = Intake(this)
    override val intakeMode: IntakeMode get() = intake.mode
    override fun intakeMode(mode: IntakeMode) {
        intake.mode = mode
    }

    override fun launch() {
        shooter.launch()
    }

    override fun getReadyForShoot() {
        shooter.getReadyForShoot()
    }

    override fun warnDriverON() {
        LedIndicator.continuousRedBlinkingON(500.milliseconds)
    }

    override fun warnDriverOFF() {
        LedIndicator.continuousRedBlinkingOFF()
    }

    override fun onLowFPS() {
        lowFPSMode = true
    }

    override fun enableFlywheel(on: Boolean) {
        shooter.enableFlywheel(on)
    }

    override fun isShooting(): Boolean {
        return shooter.stopFeederAt != 0L
    }

    override val shootingAngleCorrection: Angle
        get() = (shooter.SHOOTER_ANGLE_CORRECTION + manualAngleCorrection).degrees

    override fun loop() {
        super.loop()
        telemetry.addData("Distance to goal", goalDistance())
        telemetry.addData("Heading to goal error", headingToGoal() - currentPosition.heading)
    }

    override val artifactsCount: Int
        get() = intake.artifacts
}