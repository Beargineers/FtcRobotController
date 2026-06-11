package org.beargineers.gamma

import com.bylazar.field.FieldManager
import kotlinx.coroutines.CoroutineName
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.NonCancellable
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.beargineers.platform.Angle
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Frame
import org.beargineers.platform.FusionLocalizer
import org.beargineers.platform.LEDColor
import org.beargineers.platform.LimelightCam
import org.beargineers.platform.Localizer
import org.beargineers.platform.Location
import org.beargineers.platform.PinpointLocalizer
import org.beargineers.platform.Position
import org.beargineers.platform.RGBIndicatingRelativeLocalizer
import org.beargineers.platform.RGBIndicator
import org.beargineers.platform.RGBSignal
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.Waypoint
import org.beargineers.platform.cos
import org.beargineers.platform.decode.ArtifactsVision
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.decode.goalDistance
import org.beargineers.platform.degrees
import org.beargineers.platform.drivePath
import org.beargineers.platform.driveTo
import org.beargineers.platform.nextTick
import org.beargineers.platform.sin
import kotlin.time.Duration.Companion.milliseconds

class GammaRobot(op: RobotOpMode<DecodeRobot>) : BaseRobot(op), DecodeRobot {
    val intake = Intake(this)
    val intakeController = IntakeController(this)
    val rgb = RGBIndicator(this)
    val shooter = Shooter(this)
    val turret = Turret(this)

    val vision = ArtifactsVision(this, true)
    val ballsDetector = BallsDetector(this)

    private enum class GammaRGBSignal(override val priority: Int) : RGBSignal {
        INTAKE_TARGET(10),
        AUTO(20),
        ARTIFACTS(30),
        APRIL_TAG(40)
    }

    override fun intakeTarget(filter: (Location) -> Boolean, circularity: Double): Location? {
        return vision.calculateTargetLocation(filter, circularity)
    }

    override val localizer: Localizer =
        FusionLocalizer(
            RotatingCameraAdjuster(LimelightCam(this) {turret.turretHeading()}, turret),
            RGBIndicatingRelativeLocalizer(PinpointLocalizer(this), rgb)
        )

    private var manualAngleCorrection = 0.0
    override fun adjustShooting(distance: Double, angle: Double) {
        shooter.manualPowerAdjustment += distance
        manualAngleCorrection += angle
    }

    override val hasTurret = true
    override val hasVision = true

    override fun requestIntakeMode(mode: IntakeMode) {
        intakeController.setBaseMode(mode)
    }

    override fun requestIntakeModeOverride(mode: IntakeMode) {
        intakeController.setModeOverrideCapacity(mode)
    }

    override fun loop() {
        updateRGBIndicator()
        super.loop()
        Frame.addDevData("Distance to goal", goalDistance())
        Frame.addData("Artifacts", artifactsCount)
    }

    private var oldArtifactCount = 0
    private fun updateRGBIndicator() {
        updateArtifactIndicator()

        val autoActive = opMode.isAutoActive()
        if (autoActive) {
            rgb.glow(GammaRGBSignal.AUTO, LEDColor.WHITE)
        } else {
            rgb.clear(GammaRGBSignal.AUTO)
        }

        val intakeTargetInView = artifactsCount < 3 && intakeTarget({ true }) != null
        if (intakeTargetInView) {
            rgb.glow(GammaRGBSignal.INTAKE_TARGET, LEDColor.BLUE)
        } else {
            rgb.clear(GammaRGBSignal.INTAKE_TARGET)
        }
    }

    private fun updateArtifactIndicator() {
        if (oldArtifactCount == artifactsCount) {
            return
        }

        oldArtifactCount = artifactsCount
        when (artifactsCount) {
            3 -> rgb.glow(GammaRGBSignal.ARTIFACTS, LEDColor.GREEN)
            else -> rgb.clear(GammaRGBSignal.ARTIFACTS)
        }
    }

    private var isShooting = false
    var shootingSequenceEndedAt = 0L
    var predictedShootingPosition: Position? = null


    override suspend fun followPathAndShoot(waypoints: List<Waypoint>, applyMirroring: Boolean) {
        shootingSequenceEndedAt = System.currentTimeMillis() + 1_000_000
        try {
            coroutineScope {
                val shootingScope: CoroutineScope = this

                val delayedLatchOpen = launch {
                    delay(OPEN_LATCH_DELAY_MS.milliseconds)
                    intakeController.setShooterMode(IntakeMode.OFF)
                    shooter.openLatch()
                }

                predictedShootingPosition = waypoints.last().target
                try {
                    drivePath(waypoints, applyMirroring)
                } finally {
                    predictedShootingPosition = null
                }

                if (delayedLatchOpen.isActive) {
                    delayedLatchOpen.cancelAndJoin()
                }

                shootingJob = launch(CoroutineName("Shooting")) {
                    isShooting = true
                    holdPositionWhileShooting()

                    try {
                        shooter.shoot(shootingScope)
                    } finally {
                        isShooting = false
                        ballsDetector.reset()
                        opMode.gamepad1.rumble(300)
                    }
                }
            }
        }
        finally {
            withContext(NonCancellable) {
                intakeController.setShooterMode(null)
                shooter.closeLatch(false)
            }
            shootingSequenceEndedAt = System.currentTimeMillis()
        }

        intakeController.setBaseMode(IntakeMode.ON)
    }

    private fun CoroutineScope.holdPositionWhileShooting() {
        val initialPosition = currentPosition

        launch(CoroutineName("Hold Shooting Position")) {
            while (isShooting()) {
                driveTo(initialPosition, applyMirroring = false)
                nextTick()
            }
        }
    }

    private var shootingJob: Job? = null

    fun abortShooting() {
        shootingJob?.cancel()
        intakeController.setShooterMode(null)
        intakeController.setCapacityLimited(false)
        intakeController.reverseFor(300.milliseconds)
    }

    override suspend fun prepareForShutdown() {
        super.prepareForShutdown()
        intakeController.setBaseMode(IntakeMode.OFF)
        intakeController.setShooterMode(null)
        intakeController.setCapacityLimited(false)
        shooter.closeLatch(false)
    }

    override fun isShooting(): Boolean {
        return isShooting
    }

    override fun resetTurret() {
        turret.reset()
    }

    override val shootingAngleCorrection: Angle
        get() = (shooter.SHOOTER_ANGLE_CORRECTION + manualAngleCorrection).degrees

    override val artifactsCount: Int get() = ballsDetector.artifactsCount()

    override val shooterAngle: Angle get() = turret.turretHeading()

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
        val target = intakeTarget({ true })

        if (target != null) {
            setStyle("green", "green", 1.0)
            moveCursor(target.x.inch(), target.y.inch())
            circle(1.0)
        }
    }

    override fun shooterIsReady(): Boolean {
        return super.shooterIsReady() && shooter.isFlyWheelUpToSpeed()
    }
}
