package org.beargineers.gamma

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog
import kotlinx.coroutines.CoroutineScope
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
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.decode.flywheelEnabled
import org.beargineers.platform.decode.goalDistanceFrom
import org.beargineers.platform.decode.headingIsAtGoal
import org.beargineers.platform.decode.intakeMode
import org.beargineers.platform.doNoLongerThan
import org.beargineers.platform.motorPower
import org.beargineers.platform.nextTick
import org.beargineers.platform.roundMotorPower
import org.beargineers.platform.submitJob
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.abs
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

class Shooter(val bot: GammaRobot): Hardware(bot) {
    val SHOOTER_POWER_ADJUST by config(1.0)
    var manualPowerAdjustment = 1.0
    val SHOOTER_DISTANCE_QUOTIENT by config(0.00101)
    val SHOOTER_FREE_QUOTIENT by config(0.556)
    val SHOOTER_ERROR_MARGIN by config(0.03)
    val INITIAL_SHOOTER_ERROR_MARGIN by config(0.03)

    val SHOOTING_TIMEOUT by config(2.0)

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

    val pid = PID()

    val fly1 by hardware<DcMotorEx>()
    val fly2 by hardware<DcMotorEx>()

    val latch by hardware<Servo>()

    val pusher by hardware<Servo>()

    enum class LatchState {
        OPEN, CLOSED, OPENING, CLOSING
    }

    var latchState = CLOSED
    var pusherActive = false


    var maxTicks = 0

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
        latch.position = LATCH_SERVO_CLOSED_POSITION

        bot.submitJob("Initialize servos. Close latch, open pusher") {
            closeLatch(false)
        }
    }

    private fun powerFlywheel(p: Double) {
        fly1.motorPower = p
        fly2.motorPower = p
    }

    private fun recommendedPower(): Double {
        val p = targetFlywheelPower()
        pid.updateCoefficients(SHOOTER_PID)
        pid.setTarget(p)
        pid.updateCurrent(fly1.velocity / (maxTicks))
        Frame.graph("Shooter error", pid.error())
        Frame.graph("FW Target", p * 6000)
        Frame.graph("FW Actual", 6000 * fly1.velocity / (maxTicks))

        val v = roundMotorPower(pid.result() + p)
        return v
    }

    fun enableIntake(enable: Boolean) {
        val isCurrentlyEnabled = bot.intakeMode == IntakeMode.ON
        if (enable != isCurrentlyEnabled) {
            Frame.log(if (enable) "Intake enabled" else "Intake paused. Flywheel error: ${pid.result()}, heading is correct: ${bot.headingIsAtGoal()}.")
            bot.intakeMode = if (enable) IntakeMode.ON else IntakeMode.OFF
        }

        if (!enable) {
            bot.ballsDetector.lastSeenBall.reset()
        }
    }

    suspend fun shoot(scope: CoroutineScope) {
        openLatch()

        bot.flywheelEnabled = true
        while (bot.isShooting()) {
            if (isFlyWheelUpToSpeed() && bot.headingIsAtGoal()) {
                break
            }
            bot.nextTick()
        }

        Frame.log("Goal locked, flywheel sped up. Enabling intake")
        bot.intakeMode = IntakeMode.ON

        scope.launch {
            while (bot.isShooting()) {
                enableIntake(isFlywheelBackToSpeed() && bot.headingIsAtGoal())
                bot.nextTick()
            }
            bot.intakeMode = IntakeMode.ON
        }

        bot.doNoLongerThan(SHOOTING_TIMEOUT.seconds) {
            bot.ballsDetector.waitTillNoBalls(PUSHER_SERVO_ACTIVATION_DELAY_MS.milliseconds)
        }
        activatePusher(true)
        delay(PUSHER_SERVO_ACTIVATION_DURATION_MS.milliseconds)
        activatePusher(false)
    }

    private fun targetFlywheelPower(): Double {
        val distance = bot.goalDistanceFrom(bot.predictedShootingPosition ?: bot.currentPosition)
        return flywheelPowerAdjustedToDistance(distance.cm())
    }

    override fun loop() {
        val nominalPower = when {
            bot.flywheelEnabled -> recommendedPower()
            else -> 0.0
        }
        powerFlywheel(nominalPower)
        checkMotorsConnected()
    }

    val lastMotorChecked = ElapsedTime()
    var failedChecks = 0
    private fun checkMotorsConnected() {
        if (bot.flywheelEnabled) {
            if (lastMotorChecked.seconds() > 1) {
                lastMotorChecked.reset()

                val c1 = fly1.getCurrent(CurrentUnit.MILLIAMPS)
                val c2 = fly2.getCurrent(CurrentUnit.MILLIAMPS)

                RobotLog.i("Flywheel currents F1: $c1, F2: $c2")

                if (c1 < 4 || c2 < 4) {
                    failedChecks++

                    if (failedChecks > 3) {
                        RobotLog.clearGlobalWarningMsg()
                        RobotLog.addGlobalWarningMessage("FLYWHEEL MOTOR DISCONNECTED")
                    }
                }
                else {
                    failedChecks = 0
                }
            }
        }
    }

    // According to the experimental data this linear approximation gives coefficient of determination of 0.95. Also, voltage degradation seem to be handled quite well
    fun flywheelPowerAdjustedToDistance(distanceCm: Double): Double {
        return (SHOOTER_DISTANCE_QUOTIENT *distanceCm+ SHOOTER_FREE_QUOTIENT) * SHOOTER_POWER_ADJUST * manualPowerAdjustment
    }

    override fun stop() {
        bot.flywheelEnabled = false
        latch.position = LATCH_SERVO_CLOSED_POSITION
    }

    private var latchJob: Job? = null

    suspend fun closeLatch(waitForCompletion: Boolean) {
        activatePusher(false)

        Frame.log("Closing latch. Current state: $latchState")

        when (latchState) {
            CLOSED -> {}
            CLOSING -> if (waitForCompletion) latchJob!!.join()

            OPENING,
            OPEN -> {
                latchJob?.cancel()
                latchState = CLOSING
                latch.position = LATCH_SERVO_CLOSED_POSITION
                latchJob = bot.submitJob("Close latch") {
                    delay(LATCH_SERVO_RUN_TIME_MS.milliseconds)
                    latchState = CLOSED
                }

                if (waitForCompletion) {
                    coroutineScope {
                        latchJob!!.join()
                        if (latchState != CLOSED) cancel()
                    }
                }
            }
        }

        Frame.log("closeLatch() completed. Current state: $latchState")
    }

    suspend fun openLatch() {
        activatePusher(false)
        Frame.log("Opening latch. Current state: $latchState")

        when (latchState) {
            OPEN -> {}
            OPENING -> latchJob!!.join()

            CLOSING,
            CLOSED -> {
                latchJob?.cancel()
                latchState = OPENING
                bot.intakeMode = IntakeMode.OFF
                latch.position = LATCH_SERVO_OPEN_POSITION
                latchJob = bot.submitJob("Open latch") {
                    delay(LATCH_SERVO_RUN_TIME_MS.milliseconds)
                    latchState = OPEN
                }

                coroutineScope {
                    latchJob!!.join()
                    if (latchState != OPEN) cancel()
                }
            }
        }

        Frame.log("openLatch() completed. Latch state: $latchState")
    }

    fun activatePusher(active: Boolean) {
        Frame.log("Pusher activated: $active")
        pusherActive = active
        pusher.position = if (active) PUSHER_SERVO_CLOSED_POSITION else PUSHER_SERVO_OPEN_POSITION
    }

    fun isFlywheelBackToSpeed(): Boolean {
        return abs(pid.error()) < SHOOTER_ERROR_MARGIN
    }

    fun isFlyWheelUpToSpeed(): Boolean {
        return abs(pid.error()) < INITIAL_SHOOTER_ERROR_MARGIN
    }
}