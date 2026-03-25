package org.beargineers.gamma

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.beargineers.platform.Angle
import org.beargineers.platform.Hardware
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.abs
import org.beargineers.platform.config
import org.beargineers.platform.decode.headingToGoalFrom
import org.beargineers.platform.degrees
import kotlin.math.roundToInt

// https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-24mm-length-8mm-rex-shaft-1150-rpm-3-3-5v-encoder
private const val MOTOR_TICKS_PER_ROTATION = 145.1
private const val MOTOR_GEAR_TEETH = 16
private const val TURRET_GEAR_TEETH = 121

private const val MOTOR_TICKS_PER_ONE_TURRET_DEGREE = (MOTOR_TICKS_PER_ROTATION / 360.0) * TURRET_GEAR_TEETH / MOTOR_GEAR_TEETH

private val TURRET_AUTOROTATION_ENABLED by config(true)
private val TURRET_MOTOR_SPEED by config(1.0)
private val TURRET_MOTOR_DIRECTION by config(DcMotorSimple.Direction.REVERSE)
private val TURRET_MIN_ANGLE by config(-180.degrees)
private val TURRET_MAX_ANGLE by config(180.degrees)

private val TURRET_TICKS_LOOKAHEAD by config(1)

fun motorTicksForAngle(angle: Angle): Int {
    return (angle.degrees() * MOTOR_TICKS_PER_ONE_TURRET_DEGREE).roundToInt()
}

class Turret(val bot: GammaRobot) : Hardware(bot) {
    private val turret by hardware<DcMotorEx>()
    private var initialEncoderPosition = 0

    override fun init() {
        turret.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        turret.direction = TURRET_MOTOR_DIRECTION
        initialEncoderPosition = turret.currentPosition - motorTicksForAngle(RobotOpMode.lastKnownTurretAngle)
    }

    fun currentTurretAngle(): Angle {
        return ((turret.currentPosition - initialEncoderPosition) / MOTOR_TICKS_PER_ONE_TURRET_DEGREE).degrees
    }

    override fun loop() {
        RobotOpMode.lastKnownTurretAngle = currentTurretAngle()
        if (TURRET_AUTOROTATION_ENABLED) {
            val predictedPosition = bot.predictedPosition(TURRET_TICKS_LOOKAHEAD)
            setTurretAngle(bot.headingToGoalFrom(predictedPosition.location()) - predictedPosition.heading)
        }
    }

    private fun setTurretAngle(angle: Angle) {
        val norm = angle.normalize()
        val best = listOf(norm, norm + 360.degrees, norm - 360.degrees).filter { it in TURRET_MIN_ANGLE..TURRET_MAX_ANGLE }.minBy { abs(it - currentTurretAngle()) }
        turret.targetPosition = initialEncoderPosition + motorTicksForAngle(best)
        turret.mode = DcMotor.RunMode.RUN_TO_POSITION
        turret.power = TURRET_MOTOR_SPEED
    }

    override fun stop() {
        turret.power = 0.0
    }
}