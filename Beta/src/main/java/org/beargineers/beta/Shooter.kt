package org.beargineers.beta

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Hardware
import org.beargineers.platform.config
import org.beargineers.platform.goalDistance

class Shooter(robot: BaseRobot): Hardware(robot) {
    val SHOOTER_POWER_ADJUST by robot.config(1.0)
    // TODO: Find proper quotients for Beta
    val SHOOTER_DISTANCE_QUOTIENT by robot.config(0.00101)
    val SHOOTER_FREE_QUOTIENT by robot.config(0.556)
    val SHOOTING_TIME_SECONDS by robot.config(4.5)

    val fly1 by hardware<DcMotor>()
    val fly2 by hardware<DcMotor>()

    val feeder by hardware<DcMotor>()

    var feederStartedAt: Long = 0

    var flywheelEnabled = false
    var frozenFlywheelPower: Double? = null


    override fun init() {
        super.init()

        val launchMotors = listOf(fly1, fly2)
        fly1.direction = DcMotorSimple.Direction.FORWARD
        fly2.direction = DcMotorSimple.Direction.REVERSE
        launchMotors.forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        }

        feeder.direction = DcMotorSimple.Direction.REVERSE
        feeder.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun enableFlywheel(on: Boolean) {
        flywheelEnabled = on
    }

    private fun powerFlywheel(p: Double) {
        setMotorPower(fly1, p)
        setMotorPower(fly2, p)
    }

    fun launch() {
        frozenFlywheelPower = recommendedFlywheelPower()

        feeder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        setMotorPower(feeder, 1.0)
        feederStartedAt = System.currentTimeMillis()
    }

    private fun recommendedFlywheelPower(): Double = flywheelPowerAdjustedToDistance((this@Shooter.robot as BetaRobot).goalDistance().cm())

    override fun loop() {
        if (feederStartedAt != 0L && (System.currentTimeMillis() - feederStartedAt) > SHOOTING_TIME_SECONDS * 1000) {
            setMotorPower(feeder, 0.0)
            feederStartedAt = 0L
            frozenFlywheelPower = null
        }

        val nominalPower = when {
            flywheelEnabled -> frozenFlywheelPower ?: recommendedFlywheelPower()
            else -> 0.0
        }
        powerFlywheel(nominalPower)
    }

    // According to the experimental data this linear approximation gives coefficient of determination of 0.95. Also, voltage degradation seem to be handled quite well
    fun flywheelPowerAdjustedToDistance(distanceCm: Double): Double {
        return (SHOOTER_DISTANCE_QUOTIENT *distanceCm+ SHOOTER_FREE_QUOTIENT) * SHOOTER_POWER_ADJUST
    }

    override fun stop() {
        enableFlywheel(false)
        setMotorPower(feeder, 0.0)
    }
}