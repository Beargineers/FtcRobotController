package org.beargineers.gamma

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
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
import kotlin.math.roundToInt

//https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/
private const val MOTOR_TICKS_PER_ROTATION = 384.5

// https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-24mm-length-8mm-rex-shaft-1150-rpm-3-3-5v-encoder
// private const val MOTOR_TICKS_PER_ROTATION = 145.1

private const val MOTOR_GEAR_TEETH = 16
private const val TURRET_GEAR_TEETH = 121

private const val MOTOR_TICKS_PER_ONE_TURRET_DEGREE = (MOTOR_TICKS_PER_ROTATION / 360.0) * TURRET_GEAR_TEETH / MOTOR_GEAR_TEETH

private val TURRET_MOTOR_PIDF by config(PIDFTCoeffs(0.0, 0.0, 0.0, 0.0))
private val TURRET_MOTOR_DIRECTION by config(DcMotorSimple.Direction.REVERSE)
private val TURRET_MIN_ANGLE by config(-180.degrees)
private val TURRET_MAX_ANGLE by config(180.degrees)

private val TURRET_TICKS_LOOKAHEAD by config(1)
private val TURRET_CENTER_OFFSET by config(0.cm)

fun motorTicksForAngle(angle: Angle): Int {
    return (angle.degrees() * MOTOR_TICKS_PER_ONE_TURRET_DEGREE).roundToInt()
}

enum class TurretMode {
    OFF, FIXED, FOLLOW, SHOOTING
}
private val TURRET_MODE by config(TurretMode.FOLLOW)


class Turret(val bot: GammaRobot) : Hardware(bot) {
    private val turret by hardware<DcMotorEx>()
    private val control = PID(
        integralZone = 5.0,
        outputMin = -1.0, outputMax = 1.0
    )

    private var initialEncoderPosition = 0
    private var targetEncoderPosition = 0

    override fun init() {
        turret.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        turret.direction = TURRET_MOTOR_DIRECTION
        turret.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        if (firstTimeInitialPosition == null) {
            initialEncoderPosition = turret.currentPosition
            firstTimeInitialPosition = initialEncoderPosition
        }
        else {
            initialEncoderPosition = firstTimeInitialPosition!!
        }
    }

    fun currentTurretAngle(): Angle {
        return ((turret.currentPosition - initialEncoderPosition) / MOTOR_TICKS_PER_ONE_TURRET_DEGREE).degrees
    }

    fun turretHeading(): Angle {
        return (bot.currentPosition.heading + currentTurretAngle()).normalize()
    }

    val centerOffset get() = TURRET_CENTER_OFFSET

    override fun loop() {
        when (TURRET_MODE) {
            TurretMode.FIXED -> {
                setTurretAngle(0.degrees)
            }

            TurretMode.FOLLOW -> {
                aimAtGoal()
            }

            TurretMode.SHOOTING -> {
                if (bot.shooter.latchState != Shooter.LatchState.CLOSED) {
                    aimAtGoal()
                }
                else {
                    setTurretAngle(0.degrees)
                }
            }

            TurretMode.OFF -> {}
        }

        Frame.addDevData("Turret heading", turretHeading())
        Frame.graph("TURRET ERROR", (targetEncoderPosition - turret.currentPosition).toDouble())
    }

    private fun aimAtGoal() {
        val predictedPosition = bot.predictedShootingPosition ?: bot.predictedPosition(TURRET_TICKS_LOOKAHEAD)
        setTurretAngle(bot.headingToGoalFrom(predictedPosition.location()) - predictedPosition.heading)
    }


    private fun setTurretAngle(angle: Angle) {
        control.updateCoefficients(TURRET_MOTOR_PIDF)
        val currentTurretAngle = currentTurretAngle()
        val norm = angle.normalize()
        val best = listOf(norm, norm + 360.degrees, norm - 360.degrees).filter { it in TURRET_MIN_ANGLE..TURRET_MAX_ANGLE }.minBy { abs(it - currentTurretAngle) }
        targetEncoderPosition = initialEncoderPosition + motorTicksForAngle(best)

        control.setTarget(targetEncoderPosition.toDouble())
        control.updateCurrent(turret.currentPosition.toDouble())

        turret.motorPower = control.result()
    }

    override fun stop() {
        turret.motorPower = 0.0
    }

    fun reset() {
        initialEncoderPosition = turret.currentPosition
        firstTimeInitialPosition = initialEncoderPosition
    }

    companion object {
        var firstTimeInitialPosition: Int? = null
    }
}