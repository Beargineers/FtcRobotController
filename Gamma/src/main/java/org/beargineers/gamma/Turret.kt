package org.beargineers.gamma

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.beargineers.platform.Angle
import org.beargineers.platform.Frame
import org.beargineers.platform.Hardware
import org.beargineers.platform.PIDFTCoeffs
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.abs
import org.beargineers.platform.cm
import org.beargineers.platform.config
import org.beargineers.platform.decode.headingToGoalFrom
import org.beargineers.platform.degrees
import org.beargineers.platform.motorPower
import kotlin.math.roundToInt

// https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-24mm-length-8mm-rex-shaft-1150-rpm-3-3-5v-encoder
private const val MOTOR_TICKS_PER_ROTATION = 145.1
private const val MOTOR_GEAR_TEETH = 16
private const val TURRET_GEAR_TEETH = 121

private const val MOTOR_TICKS_PER_ONE_TURRET_DEGREE = (MOTOR_TICKS_PER_ROTATION / 360.0) * TURRET_GEAR_TEETH / MOTOR_GEAR_TEETH

private val TURRET_AUTOROTATION_ENABLED by config(true)
private val TURRET_MOTOR_SPEED by config(1.0)
private val TURRET_MOTOR_PIDF by config(PIDFTCoeffs(0.0, 0.0, 0.0, 0.0))
private val TURRET_MOTOR_P by config(0.0)
private val TURRET_MOTOR_DIRECTION by config(DcMotorSimple.Direction.REVERSE)
private val TURRET_MIN_ANGLE by config(-180.degrees)
private val TURRET_MAX_ANGLE by config(180.degrees)

private val TURRET_TICKS_LOOKAHEAD by config(1)
private val TURRET_CENTER_OFFSET by config(0.cm)

fun motorTicksForAngle(angle: Angle): Int {
    return (angle.degrees() * MOTOR_TICKS_PER_ONE_TURRET_DEGREE).roundToInt()
}

class Turret(val bot: GammaRobot) : Hardware(bot) {
    private val turret by hardware<DcMotorEx>()
    private val magnet by hardware<DigitalChannel>()
    private var initialEncoderPosition = 0
    private var targetEncoderPosition = 0

    override fun init() {
        turret.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        turret.direction = TURRET_MOTOR_DIRECTION
        if (firstTimeInitialPosition == null) {
            initialEncoderPosition = turret.currentPosition
            firstTimeInitialPosition = initialEncoderPosition
        }
        else {
            initialEncoderPosition = firstTimeInitialPosition!!
        }
    }

    fun assumeAngle(angle: Angle) {
        //initialEncoderPosition = turret.currentPosition - motorTicksForAngle(angle)
    }

    fun currentTurretAngle(): Angle {
        return ((turret.currentPosition - initialEncoderPosition) / MOTOR_TICKS_PER_ONE_TURRET_DEGREE).degrees
    }

    val centerOffset get() = TURRET_CENTER_OFFSET

    override fun loop() {
        Frame.graph("TURRET ERROR", 0.0 + targetEncoderPosition - turret.currentPosition)
        RobotOpMode.lastKnownTurretAngle = currentTurretAngle()
        if (TURRET_AUTOROTATION_ENABLED) {
            val predictedPosition = bot.predictedPosition(TURRET_TICKS_LOOKAHEAD)
            setTurretAngle(bot.headingToGoalFrom(predictedPosition.location()) - predictedPosition.heading)
        }
    }

    private fun setTurretAngle(angle: Angle) {
        if (TURRET_MOTOR_PIDF.p > 0) {
            turret.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, with (TURRET_MOTOR_PIDF) {
                PIDFCoefficients(p, i, d, t)
            })
        }

        if (TURRET_MOTOR_P > 0) {
            turret.setPositionPIDFCoefficients(TURRET_MOTOR_P)
        }

        val norm = angle.normalize()
        val best = listOf(norm, norm + 360.degrees, norm - 360.degrees).filter { it in TURRET_MIN_ANGLE..TURRET_MAX_ANGLE }.minBy { abs(it - currentTurretAngle()) }
        targetEncoderPosition = initialEncoderPosition + motorTicksForAngle(best)
        turret.targetPosition = targetEncoderPosition
        turret.mode = DcMotor.RunMode.RUN_TO_POSITION
        turret.motorPower = TURRET_MOTOR_SPEED
    }

    override fun stop() {
        turret.motorPower = 0.0
    }

    companion object {
        var firstTimeInitialPosition: Int? = null
    }
}