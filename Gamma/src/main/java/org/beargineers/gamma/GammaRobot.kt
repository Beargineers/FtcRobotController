package org.beargineers.gamma

import org.beargineers.platform.Angle
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.FusionLocalizer
import org.beargineers.platform.LimelightCam
import org.beargineers.platform.Localizer
import org.beargineers.platform.PinpointLocalizer
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.decode.headingToGoal
import org.beargineers.platform.degrees

class GammaRobot(op: RobotOpMode<DecodeRobot>) : BaseRobot(op), DecodeRobot {
    val intake = Intake(this)
    val shooter = Shooter(this)
    val turret = Turret(this)

    override val localizer: Localizer =
        FusionLocalizer(
            LimelightCam(this),
            PinpointLocalizer(this)
        )

    override val intakeMode: IntakeMode get() = intake.mode
    override fun intakeMode(mode: IntakeMode) {
        intake.mode = mode
    }

    private var manualAngleCorrection = 0.0
    override fun adjustShooting(distance: Double, angle: Double) {
        shooter.manualPowerAdjustment += distance
        manualAngleCorrection += angle
    }

    override val hasTurret = true

    override fun launch() {
        shooter.launch()
    }

    override fun prepareForShooting() {
        // TODO Anything?
    }

    override fun enableFlywheel(on: Boolean) {
        shooter.enableFlywheel(on)
    }

    override fun isShooting(): Boolean {
        return shooter.isShooting()
    }

    override val shootingAngleCorrection: Angle
        get() = (shooter.SHOOTER_ANGLE_CORRECTION + manualAngleCorrection).degrees

    override val artifactsCount: Int get() = 0 // TODO

    override fun loop() {
        super.loop()
        turret.setTurretAngle(headingToGoal() - currentPosition.heading)
    }
}