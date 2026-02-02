package org.beargineers.beta

import org.beargineers.platform.Angle
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.FusionLocalizer
import org.beargineers.platform.IndicatingRelativeLocalizer
import org.beargineers.platform.LedIndicator
import org.beargineers.platform.LimelightCam
import org.beargineers.platform.Localizer
import org.beargineers.platform.MecanumDrive
import org.beargineers.platform.PinpointLocalizer
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.blink
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.decode.goalDistance
import org.beargineers.platform.decode.headingToGoal
import org.beargineers.platform.degrees
import kotlin.time.Duration.Companion.seconds

class BetaRobot(op: RobotOpMode<DecodeRobot>) : BaseRobot(op), DecodeRobot {
    var lowFPSMode = false
    private var manualAngleCorrection = 0.0
    override val drive = MecanumDrive(this)
    override fun adjustShooting(distance: Double, angle: Double) {
        shooter.manualPowerAdjustment += distance
        manualAngleCorrection += angle
    }

    val ledIndicator = LedIndicator(this)

    override val localizer: Localizer =
        FusionLocalizer(telemetry,
            LimelightCam(this),
            IndicatingRelativeLocalizer(PinpointLocalizer(this), ledIndicator)
        )

    val shooter = Shooter(this)

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
        ledIndicator.setTempPattern(blink("RRR"), 2.seconds)
    }

    override fun lowFpsMode(mode: Boolean) {
        if (mode && !lowFPSMode) {
            lowFPSMode = true
            ledIndicator.setBasePattern(blink("RGR"))
        }
        else if (!mode) {
            lowFPSMode = false
        }
    }

    override fun enableFlywheel(on: Boolean) {
        shooter.enableFlywheel(on)
    }

    override fun isShooting(): Boolean {
        return shooter.isShooting()
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