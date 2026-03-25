package org.beargineers.gamma

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.Frame
import org.beargineers.platform.Hardware
import org.beargineers.platform.PID
import org.beargineers.platform.PIDFTCoeffs
import org.beargineers.platform.config
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.goalDistance
import org.beargineers.platform.roundMotorPower

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

    val pid = PID()

    val fly1 by hardware<DcMotor>()
    val fly2 by hardware<DcMotor>()

    val latch by hardware<Servo>()

    var flywheelEnabled = false

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
        latch.direction = Servo.Direction.REVERSE
    }

    fun enableFlywheel(on: Boolean) {
        flywheelEnabled = on

        if (on) {
            latch.position = LATCH_SERVO_OPEN_POSITION
        }
        else {1
            latch.position = LATCH_SERVO_CLOSED_POSITION
        }

    }

    private fun powerFlywheel(p: Double) {
        pid.updateCoefficients(SHOOTER_PID)
        pid.setTarget(p)
        pid.updateCurrent((fly1 as DcMotorEx).velocity / (maxTicks))
        Frame.addData("Shooter error", pid.error())

        val v = roundMotorPower(pid.result() + p)
        fly1.power = v
        fly2.power = v
    }

    fun launch() {
        // TODO open latch
        shootingTime.reset()
        bot.intake.onShoot()
    }

    private fun recommendedFlywheelPower(): Double = flywheelPowerAdjustedToDistance((this@Shooter.robot as DecodeRobot).goalDistance().cm())

    override fun loop() {
        val nominalPower = when {
            flywheelEnabled -> recommendedFlywheelPower()
            else -> 0.0
        }
        powerFlywheel(nominalPower)
    }

    // According to the experimental data this linear approximation gives coefficient of determination of 0.95. Also, voltage degradation seem to be handled quite well
    fun flywheelPowerAdjustedToDistance(distanceCm: Double): Double {
        return (SHOOTER_DISTANCE_QUOTIENT *distanceCm+ SHOOTER_FREE_QUOTIENT) * SHOOTER_POWER_ADJUST * manualPowerAdjustment
    }

    override fun stop() {
        enableFlywheel(false)
    }

    fun isShooting(): Boolean {
        return shootingTime.seconds() < SHOOTING_TIME_SECONDS
    }
}