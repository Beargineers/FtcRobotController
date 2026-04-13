package org.beargineers.gamma

import com.bylazar.field.FieldManager
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.launch
import org.beargineers.platform.Angle
import org.beargineers.platform.ArtifactsVision
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Frame
import org.beargineers.platform.FusionLocalizer
import org.beargineers.platform.IndicatingRelativeLocalizer
import org.beargineers.platform.LedIndicator
import org.beargineers.platform.LimelightCam
import org.beargineers.platform.Localizer
import org.beargineers.platform.Location
import org.beargineers.platform.PinpointLocalizer
import org.beargineers.platform.Position
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.cos
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.decode.goalDistance
import org.beargineers.platform.decode.intakeMode
import org.beargineers.platform.degrees
import org.beargineers.platform.driveTo
import org.beargineers.platform.nextTick
import org.beargineers.platform.sin

class GammaRobot(op: RobotOpMode<DecodeRobot>) : BaseRobot(op), DecodeRobot {
    val intake = Intake(this)
    val shooter = Shooter(this)
    val turret = Turret(this)
    val ledIndicator = LedIndicator(2, this)

    val vision = ArtifactsVision(this, true)
    val ballsDetector = BallsDetector(this)

    override fun intakeTarget(filter: (Location) -> Boolean): Location? {
        return vision.calculateTargetLocation(filter)
    }

    override val localizer: Localizer =
        FusionLocalizer(
            RotatingCameraAdjuster(LimelightCam(this), turret),
            IndicatingRelativeLocalizer(PinpointLocalizer(this), ledIndicator)
        )

    private var manualAngleCorrection = 0.0
    override fun adjustShooting(distance: Double, angle: Double) {
        shooter.manualPowerAdjustment += distance
        manualAngleCorrection += angle
    }

    override val hasTurret = true
    override val hasVision = true

    override fun loop() {
        super.loop()
        Frame.addData("Distance to goal", goalDistance())
        Frame.addData("Artifacts", artifactsCount)
        ledIndicator.counter(artifactsCount, 'G')
    }

    override suspend fun shoot(holdPosition: Boolean) {
        coroutineScope {
            val hold = launch {
                val initialPosition = currentPosition
                do {
                    nextTick()
                    if (holdPosition) {
                        driveTo(initialPosition, applyMirroring = false)
                    }
                } while (isShooting())
            }

            shooter.startShooting()
            hold.join()

            shooter.closeLatch(false)
            intakeMode = IntakeMode.ON
            ballsDetector.reset()
            opMode.gamepad1.rumble(300)
        }
    }

    override suspend fun prepareForShooting() {
        shooter.openLatch()
    }

    override fun isShooting(): Boolean {
        return shooter.isShooting()
    }

    override fun assumePosition(position: Position, turretAngle: Angle) {
        super.assumePosition(position, turretAngle)
        turret.assumeAngle(turretAngle)
    }

    override fun resetTurret() {
        turret.reset()
    }

    override val shootingAngleCorrection: Angle
        get() = (shooter.SHOOTER_ANGLE_CORRECTION + manualAngleCorrection).degrees

    override val artifactsCount: Int get() = ballsDetector.artifactsCount()

    override val shooterAngle: Angle get() = currentPosition.heading + turret.currentTurretAngle()

    override fun FieldManager.drawExtraFeatures() {
        drawVisualTarget()
        drawTurret()
    }

    private fun FieldManager.drawTurret() {
        val cp = currentPosition
        setStyle("white", "white", 1.0)
        moveCursor(cp.x.inch(), cp.y.inch())
        circle(2.0)

        moveCursor(cp.x.inch() + 3 * cos(shooterAngle), cp.y.inch() + 3 * sin(shooterAngle))
        circle(1.0)
    }

    private fun FieldManager.drawVisualTarget() {
        val target = intakeTarget { true }

        if (target != null) {
            setStyle("green", "green", 1.0)
            moveCursor(target.x.inch(), target.y.inch())
            circle(1.0)
        }
    }
}