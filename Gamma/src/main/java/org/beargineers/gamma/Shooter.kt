package org.beargineers.gamma

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.Job
import kotlinx.coroutines.cancel
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.beargineers.gamma.Shooter.LatchState.CLOSED
import org.beargineers.gamma.Shooter.LatchState.CLOSING
import org.beargineers.gamma.Shooter.LatchState.OPEN
import org.beargineers.gamma.Shooter.LatchState.OPENING
import org.beargineers.platform.Frame
import org.beargineers.platform.Hardware
import org.beargineers.platform.PID
import org.beargineers.platform.PIDFTCoeffs
import org.beargineers.platform.config
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.decode.flywheelEnabled
import org.beargineers.platform.decode.goalDistance
import org.beargineers.platform.decode.intakeMode
import org.beargineers.platform.roundMotorPower
import kotlin.math.abs
import kotlin.time.Duration.Companion.milliseconds

class Shooter(val bot: GammaRobot): Hardware(bot) {
    val SHOOTER_POWER_ADJUST by config(1.0)
    var manualPowerAdjustment = 1.0
    val SHOOTER_DISTANCE_QUOTIENT by config(0.00101)
    val SHOOTER_FREE_QUOTIENT by config(0.556)
    val SHOOTING_TIME_SECONDS by config(4.5)
    val SHOOTER_ERROR_MARGIN by config(0.03)

    val SHOOTER_PID by config(PIDFTCoeffs(0.0, 0.0, 0.0, 0.0))
    val SHOOTER_ANGLE_CORRECTION by config(0.0)

    val LATCH_SERVO_OPEN_POSITION by config(0.30)
    val LATCH_SERVO_CLOSED_POSITION by config(0.30)
    val LATCH_SERVO_RUN_TIME_MS by config(500)
    val LATCH_SERVO_DIRECTION by config(DcMotorSimple.Direction.FORWARD)

    val PUSHER_SERVO_OPEN_POSITION by config(0.0)
    val PUSHER_SERVO_CLOSED_POSITION by config(1.0)
    val PUSHER_SERVO_ACTIVATION_DELAY_MS by config(50)
    val PUSHER_SERVO_ACTIVATION_DURATION_MS by config(100)
    val PUSHER_SERVO_RELEASE_THRESHOLD_MA by config(1800)

    val pid = PID()

    val fly1 by hardware<DcMotor>()
    val fly2 by hardware<DcMotor>()

    val latch by hardware<Servo>()
    val pusher by hardware<Servo>()

    enum class LatchState {
        OPEN, CLOSED, OPENING, CLOSING
    }

    var latchState = OPEN

    var maxTicks = 0

    val shootingTime = ElapsedTime()

    override fun init() {
        super.init()

        val launchMotors = listOf(fly1, fly2)
        fly1.direction = DcMotorSimple.Direction.REVERSE
        fly2.direction = DcMotorSimple.Direction.FORWARD
        launchMotors.forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        }

        maxTicks = fly1.motorType.achieveableMaxTicksPerSecondRounded
        latch.direction = when (LATCH_SERVO_DIRECTION) {
            DcMotorSimple.Direction.FORWARD -> Servo.Direction.FORWARD
            DcMotorSimple.Direction.REVERSE -> Servo.Direction.REVERSE
        }

        pusher.direction = Servo.Direction.FORWARD
        activatePusher(false)
    }

    private fun powerFlywheel(p: Double) {
        fly1.power = p
        fly2.power = p
    }

    private fun recommendedPower(): Double {
        val p = targetFlywheelPower()
        pid.updateCoefficients(SHOOTER_PID)
        pid.setTarget(p)
        pid.updateCurrent((fly1 as DcMotorEx).velocity / (maxTicks))
        Frame.addData("Shooter error", pid.error())

        val v = roundMotorPower(pid.result() + p)
        return v
    }

    suspend fun shoot() {
        coroutineScope {
            openLatch()
            waitForFlywheelSpeed()

            launch {
                activatePusher(false)
                delay(PUSHER_SERVO_ACTIVATION_DELAY_MS.milliseconds)
                activatePusher(true)
                delay(PUSHER_SERVO_ACTIVATION_DURATION_MS.milliseconds)
                activatePusher(false)
            }

            shootingTime.reset()
            bot.intakeMode = IntakeMode.ON
        }
    }

    private suspend fun waitForFlywheelSpeed() {
        while (abs(pid.error()) > SHOOTER_ERROR_MARGIN) {
            bot.opMode.yield()
        }
    }

    private fun targetFlywheelPower(): Double = flywheelPowerAdjustedToDistance((this@Shooter.robot as DecodeRobot).goalDistance().cm())

    override fun loop() {
        val nominalPower = when {
            bot.flywheelEnabled -> recommendedPower()
            else -> 0.0
        }
        powerFlywheel(nominalPower)
    }

    // According to the experimental data this linear approximation gives coefficient of determination of 0.95. Also, voltage degradation seem to be handled quite well
    fun flywheelPowerAdjustedToDistance(distanceCm: Double): Double {
        return (SHOOTER_DISTANCE_QUOTIENT *distanceCm+ SHOOTER_FREE_QUOTIENT) * SHOOTER_POWER_ADJUST * manualPowerAdjustment
    }

    override fun stop() {
        bot.flywheelEnabled = false
    }

    fun isShooting(): Boolean {
        return shootingTime.seconds() < SHOOTING_TIME_SECONDS
    }

    private var latchJob: Job? = null

    suspend fun closeLatch(waitForCompletion: Boolean) {
        activatePusher(false)

        when (latchState) {
            CLOSED -> {}
            CLOSING -> latchJob!!.join()

            OPENING,
            OPEN -> {
                latchJob?.cancel()
                latchState = CLOSING
                coroutineScope {
                    latchJob = launch {
                        bot.intakeMode = IntakeMode.OFF
                        latch.position = LATCH_SERVO_CLOSED_POSITION
                        delay(LATCH_SERVO_RUN_TIME_MS.milliseconds)
                        latchState = CLOSED
                    }

                    if (waitForCompletion) {
                        latchJob!!.join()
                        if (latchState != CLOSED) cancel()
                    }
                }
            }
        }
    }

    suspend fun openLatch() {
        when (latchState) {
            OPEN -> {}
            OPENING -> latchJob!!.join()

            CLOSING,
            CLOSED -> {
                latchJob?.cancel()
                latchState = OPENING
                coroutineScope {
                    latchJob = launch {
                        bot.intakeMode = IntakeMode.OFF
                        latch.position = LATCH_SERVO_OPEN_POSITION
                        delay(LATCH_SERVO_RUN_TIME_MS.milliseconds)
                        latchState = OPEN
                    }

                    latchJob!!.join()
                    if (latchState != OPEN) cancel()
                }
            }
        }
    }

    fun activatePusher(active: Boolean) {
        pusher.position = if (active) PUSHER_SERVO_CLOSED_POSITION else PUSHER_SERVO_OPEN_POSITION
    }
}