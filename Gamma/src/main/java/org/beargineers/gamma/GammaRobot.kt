package org.beargineers.gamma

import com.bylazar.field.FieldManager
import kotlinx.coroutines.CoroutineName
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.NonCancellable
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
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
import org.beargineers.platform.Waypoint
import org.beargineers.platform.cos
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.decode.goalDistance
import org.beargineers.platform.decode.intakeMode
import org.beargineers.platform.degrees
import org.beargineers.platform.drivePath
import org.beargineers.platform.driveTo
import org.beargineers.platform.nextTick
import org.beargineers.platform.sin
import org.beargineers.platform.submitJob
import kotlin.time.Duration.Companion.milliseconds

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
            RotatingCameraAdjuster(LimelightCam(this) {turret.turretHeading()}, turret),
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
        Frame.addDevData("Distance to goal", goalDistance())
        Frame.addData("Artifacts", artifactsCount)
        ledIndicator.counter(artifactsCount, 'G')
    }

    private var isShooting = false
    var predictedShootingPosition: Position? = null


    override suspend fun followPathAndShoot(waypoints: List<Waypoint>, applyMirroring: Boolean) {
        try {
            coroutineScope {
                val shootingScope: CoroutineScope = this

                launch {
                    delay(OPEN_LATCH_DELAY_MS.milliseconds)
                    shooter.openLatch()
                }

                predictedShootingPosition = waypoints.last().target
                try {
                    drivePath(waypoints, applyMirroring)
                } finally {
                    predictedShootingPosition = null
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
                shooter.closeLatch(false)
            }
        }

        intakeMode = IntakeMode.ON
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
        submitJob("Reversing intake on aborted shooting") {
            intakeMode = IntakeMode.REVERSE
            delay(300.milliseconds)
            intakeMode = IntakeMode.ON
        }
    }

    override suspend fun prepareForShutdown() {
        super.prepareForShutdown()
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
        val target = intakeTarget { true }

        if (target != null) {
            setStyle("green", "green", 1.0)
            moveCursor(target.x.inch(), target.y.inch())
            circle(1.0)
        }
    }

    override fun shooterIsReady(): Boolean {
        return super.shooterIsReady() && shooter.isUpToSpeed()
    }
}