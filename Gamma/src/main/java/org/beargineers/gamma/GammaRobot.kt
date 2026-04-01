package org.beargineers.gamma

import org.beargineers.platform.Angle
import org.beargineers.platform.ArtifactsVision
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.FusionLocalizer
import org.beargineers.platform.LimelightCam
import org.beargineers.platform.Localizer
import org.beargineers.platform.Location
import org.beargineers.platform.PinpointLocalizer
import org.beargineers.platform.Position
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.degrees

class GammaRobot(op: RobotOpMode<DecodeRobot>) : BaseRobot(op), DecodeRobot {
    val intake = Intake(this)
    val shooter = Shooter(this)
    val turret = Turret(this)

    val vision = ArtifactsVision(this, true)

    override fun intakeTarget(filter: (Location) -> Boolean): Location? {
        return vision.calculateTargetLocation(filter)
    }

    override val localizer: Localizer =
        FusionLocalizer(
            RotatingCameraAdjuster(LimelightCam(this), turret),
            PinpointLocalizer(this)
        )

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

    override fun isShooting(): Boolean {
        return shooter.isShooting()
    }

    override fun assumePosition(position: Position, turretAngle: Angle) {
        super.assumePosition(position, turretAngle)
        turret.assumeAngle(turretAngle)
    }

    override val shootingAngleCorrection: Angle
        get() = (shooter.SHOOTER_ANGLE_CORRECTION + manualAngleCorrection).degrees

    override val artifactsCount: Int get() = intake.artifactsCount

    override fun doDrawRobot() {
        super.doDrawRobot()

        val target = intakeTarget { true }

        if (target != null) {
            with(panelsField) {
                setStyle("green", "green", 1.0)
                moveCursor(target.x.inch(), target.y.inch())
                circle(1.0)
            }
        }
    }
}