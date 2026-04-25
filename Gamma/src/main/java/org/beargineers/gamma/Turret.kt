package org.beargineers.gamma

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor.IntegrationControl
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDeviceHealth
import org.beargineers.platform.Angle
import org.beargineers.platform.Frame
import org.beargineers.platform.Hardware
import org.beargineers.platform.PID
import org.beargineers.platform.PIDFTCoeffs
import org.beargineers.platform.abs
import org.beargineers.platform.cm
import org.beargineers.platform.config
import org.beargineers.platform.decode.headingToGoalFrom
import org.beargineers.platform.degrees
import org.beargineers.platform.motorPower
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.roundToInt

// https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-24mm-length-8mm-rex-shaft-1150-rpm-3-3-5v-encoder
private const val MOTOR_TICKS_PER_ROTATION = 145.1
private const val MOTOR_GEAR_TEETH = 16
private const val TURRET_GEAR_TEETH = 121

private const val MOTOR_TICKS_PER_ONE_TURRET_DEGREE = (MOTOR_TICKS_PER_ROTATION / 360.0) * TURRET_GEAR_TEETH / MOTOR_GEAR_TEETH

private val TURRET_AUTOROTATION_ENABLED by config(true)
private val NAVX_ENABLED by config(false)

private val TURRET_MOTOR_PIDF by config(PIDFTCoeffs(0.0, 0.0, 0.0, 0.0))
private val TURRET_MOTOR_DIRECTION by config(DcMotorSimple.Direction.REVERSE)
private val TURRET_MIN_ANGLE by config(-180.degrees)
private val TURRET_MAX_ANGLE by config(180.degrees)

private val TURRET_TICKS_LOOKAHEAD by config(1)
private val TURRET_CENTER_OFFSET by config(0.cm)

private val NAVX_yawScalar by config(1.0)

fun motorTicksForAngle(angle: Angle): Int {
    return (angle.degrees() * MOTOR_TICKS_PER_ONE_TURRET_DEGREE).roundToInt()
}

class Turret(val bot: GammaRobot) : Hardware(bot) {
    private val turret by hardware<DcMotorEx>()
    private val navx by hardware<NavxMicroNavigationSensor>()

    private val control = PID(
        integralZone = 5.0,
        outputMin = -1.0, outputMax = 1.0
    )

    private var currentHeading = 0.degrees

    private var initialEncoderPosition = 0
    private var initialNavXHeading = 0.degrees
    private var targetEncoderPosition = 0

    override fun init() {
        turret.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        turret.direction = TURRET_MOTOR_DIRECTION
        turret.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        var attempts = 0
        while (navx.isCalibrating() && attempts++ < 50) {
            Thread.sleep(50)
        }

        if (firstTimeInitialPosition == null) {
            firstTimeInitialPosition = turret.currentPosition
        }
        initialEncoderPosition = firstTimeInitialPosition!!
        initialNavXHeading = (bot.currentPosition.heading + currentTurretAngle()).normalize()
    }

    private var initialized = false
    private var previousYawDeg = 0.0
    private var accumulatedYawDeg = 0.0

    private fun navxHeading(): Angle {
        val o = navx.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)

        val yawDeg = o.firstAngle.toDouble() // Z angle, wrapped -180..+180

        if (!initialized) {
            previousYawDeg = yawDeg
            initialized = true
            return yawDeg.degrees.normalize()
        }

        var delta = yawDeg - previousYawDeg

        // Unwrap crossing +180/-180
        if (delta > 180.0) {
            delta -= 360.0
        } else if (delta < -180.0) {
            delta += 360.0
        }

        accumulatedYawDeg += delta
        previousYawDeg = yawDeg
        return (accumulatedYawDeg * NAVX_yawScalar).degrees.normalize()
    }

    fun currentTurretAngle(): Angle {
        return ((turret.currentPosition - initialEncoderPosition) / MOTOR_TICKS_PER_ONE_TURRET_DEGREE).degrees
    }

    fun turretHeading(): Angle {
        return if (isNavxHealthy())
            currentHeading
        else
            (bot.currentPosition.heading + currentTurretAngle()).normalize()
    }

    val centerOffset get() = TURRET_CENTER_OFFSET

    override fun loop() {
        if (isNavxHealthy()) {
            currentHeading = (initialNavXHeading + navxHeading()).normalize()

            if (!bot.isMoving() && control.error() <= 1) {
                applyTurretCorrection()
            }
        }

        if (TURRET_AUTOROTATION_ENABLED) {
            val predictedPosition = bot.predictedPosition(TURRET_TICKS_LOOKAHEAD)
            setTurretAngle(bot.headingToGoalFrom(predictedPosition.location()) - predictedPosition.heading)
        }

        Frame.addData("Turret heading", turretHeading())
        Frame.graph("TURRET ERROR", (targetEncoderPosition - turret.currentPosition).toDouble())
    }

    private fun applyTurretCorrection() {
        val headingViaEncoder = bot.currentPosition.heading + currentTurretAngle()
        val headingError = (headingViaEncoder - currentHeading).normalize()
        val ticksCorrection1 = motorTicksForAngle(headingError)
        val ticksCorrection = ticksCorrection1

        if (abs(ticksCorrection) > 1) { // Avoid jittering if correction could have been just a result of rounding
            Frame.log("Ticks correction: $ticksCorrection")
            initialEncoderPosition += ticksCorrection.coerceIn(-5, +5)
        }
    }

    private fun isNavxHealthy(): Boolean {
        if (!NAVX_ENABLED) return false

        val status = navx.deviceClient.healthStatus
        return status == HardwareDeviceHealth.HealthStatus.HEALTHY || status == HardwareDeviceHealth.HealthStatus.UNKNOWN
    }

    private fun setTurretAngle(angle: Angle) {
        control.updateCoefficients(TURRET_MOTOR_PIDF)
        val norm = angle.normalize()
        val best = listOf(norm, norm + 360.degrees, norm - 360.degrees).filter { it in TURRET_MIN_ANGLE..TURRET_MAX_ANGLE }.minBy { abs(it - currentTurretAngle()) }
        targetEncoderPosition = initialEncoderPosition + motorTicksForAngle(best)

        control.setTarget(targetEncoderPosition.toDouble())
        control.updateCurrent(turret.currentPosition.toDouble())

        turret.motorPower = control.result()
    }

    override fun stop() {
        turret.motorPower = 0.0
    }

    fun reset() {
        navx.write8(NavxMicroNavigationSensor.Register.INTEGRATION_CTL, IntegrationControl.RESET_ALL.bVal);
        initialized = false;
        previousYawDeg = 0.0;
        accumulatedYawDeg = 0.0;

        initialNavXHeading = bot.currentPosition.heading
        firstTimeInitialPosition = turret.currentPosition
        initialEncoderPosition = firstTimeInitialPosition!!
    }

    companion object {
        var firstTimeInitialPosition: Int? = null
    }
}