package org.beargineers.beta

import kotlinx.coroutines.Job
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
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
import org.beargineers.platform.Waypoint
import org.beargineers.platform.blink
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.decode.goalDistance
import org.beargineers.platform.decode.headingToGoal
import org.beargineers.platform.decode.headingToGoalFrom
import org.beargineers.platform.decode.intakeMode
import org.beargineers.platform.degrees
import org.beargineers.platform.drivePath
import org.beargineers.platform.driveTo
import org.beargineers.platform.nextTick
import kotlin.time.Duration.Companion.seconds

class BetaRobot(op: RobotOpMode<DecodeRobot>) : BaseRobot(op), DecodeRobot {
    private var lowFPSMode = false
    private var manualAngleCorrection = 0.0
    override fun adjustShooting(distance: Double, angle: Double) {
        shooter.manualPowerAdjustment += distance
        manualAngleCorrection += angle
    }

    override val hasTurret = false

    val ledIndicator = LedIndicator(3,this)

    override val localizer: Localizer =
        FusionLocalizer(
            LimelightCam(this) {currentPosition.heading},
            IndicatingRelativeLocalizer(PinpointLocalizer(this), ledIndicator)
        )

    val shooter = Shooter(this)

    val intake = Intake(this)

    var prepareJob: Job? = null

    override suspend fun followPathAndShoot(waypoints: List<Waypoint>, applyMirroring: Boolean) {
        coroutineScope {
            launch {
                prepareForShooting()
            }

            drivePath(waypoints, applyMirroring)

            shoot(true)
        }
    }

    suspend fun shoot(holdPosition: Boolean) {
        prepareJob?.cancel()

        val cl = currentPosition.location()
        shooter.launch()

        coroutineScope {
            val hold = launch {
                while (true) {
                    if (holdPosition) {
                        driveTo(cl.withHeading(headingToGoalFrom(cl)), applyMirroring = false)
                    }
                    opMode.nextTick()
                }
            }

            while (isShooting()) {
                nextTick()
            }

            hold.cancel()
        }
    }

    suspend fun prepareForShooting() {
        coroutineScope {
            prepareJob = launch {
                delay(0.2.seconds)
                intakeMode = IntakeMode.ON
                delay(0.5.seconds)

                shooter.getReadyForShoot()
            }
        }
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

    override fun shooterIsReady(): Boolean {
        return super.shooterIsReady() && shooter.isUpToSpeed()
    }
}