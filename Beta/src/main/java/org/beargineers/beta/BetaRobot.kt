package org.beargineers.beta

import org.beargineers.platform.Angle
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Frame
import org.beargineers.platform.FusionLocalizer
import org.beargineers.platform.IndicatingRelativeLocalizer
import org.beargineers.platform.LedIndicator
import org.beargineers.platform.LimelightCam
import org.beargineers.platform.Localizer
import org.beargineers.platform.PinpointLocalizer
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.blink
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.goalDistance
import org.beargineers.platform.decode.headingToGoal
import org.beargineers.platform.degrees

class BetaRobot(op: RobotOpMode<DecodeRobot>) : BaseRobot(op), DecodeRobot {
    private var lowFPSMode = false
    private var manualAngleCorrection = 0.0
    override fun adjustShooting(distance: Double, angle: Double) {
        shooter.manualPowerAdjustment += distance
        manualAngleCorrection += angle
    }

    override val hasTurret = false

    val ledIndicator = LedIndicator(this)

    override val localizer: Localizer =
        FusionLocalizer(
            LimelightCam(this),
            IndicatingRelativeLocalizer(PinpointLocalizer(this), ledIndicator)
        )

    val shooter = Shooter(this)

    val intake = Intake(this)

    override fun launch() {
        shooter.launch()
    }

    override fun prepareForShooting() {
        shooter.getReadyForShoot()
    }

    override fun isShooting(): Boolean {
        return shooter.isShooting()
    }

    override val shootingAngleCorrection: Angle
        get() = (shooter.SHOOTER_ANGLE_CORRECTION + manualAngleCorrection).degrees

    override fun loop() {
        super.loop()
        val mode = opMode.isFpsLow()
        if (mode && !lowFPSMode) {
            lowFPSMode = true
            ledIndicator.setBasePattern(blink("RGR"))
        } else if (!mode) {
            lowFPSMode = false
        }

        Frame.addData("Distance to goal", goalDistance())
        Frame.addData("Heading to goal error", headingToGoal() - currentPosition.heading)
    }

    override val artifactsCount: Int
        get() = intake.artifacts
}