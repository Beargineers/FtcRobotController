package org.beargineers

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Hardware

@Configurable
object ShooterConfig {
    var SHOOTER_POWER_ADJUST = 1.0
    var SHOOTER_DISTANCE_QUOTIENT = 0.00101
    var SHOOTER_FREE_QUOTIENT = 0.556
    var SHOOTING_TIME_SECONDS: Double = 4.5
}

class Shooter(op: BaseRobot): Hardware(op) {
    val fly1 by hardware<DcMotor>()
    val fly2 by hardware<DcMotor>()

    val feeder by hardware<DcMotor>()

    var feederStartedAt: Long = 0

    var flywheelEnabled = false
    var frozenFlywheelPower: Double? = null

    var defaultGoalDistance: Double = 90.0

    override fun init() {
        super.init()

        val launchMotors = listOf(fly1, fly2)

        launchMotors.forEach {
            it.direction = DcMotorSimple.Direction.REVERSE
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
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

    private fun recommendedFlywheelPower(): Double = flywheelPowerAdjustedToDistance((robot as DecodeRobot).goalDistanceCM ?: defaultGoalDistance)

    override fun loop() {
        if (feederStartedAt != 0L && (System.currentTimeMillis() - feederStartedAt) > ShooterConfig.SHOOTING_TIME_SECONDS * 1000) {
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
        return (ShooterConfig.SHOOTER_DISTANCE_QUOTIENT *distanceCm+ ShooterConfig.SHOOTER_FREE_QUOTIENT) * ShooterConfig.SHOOTER_POWER_ADJUST
    }

    override fun stop() {
        enableFlywheel(false)
        setMotorPower(feeder, 0.0)
    }
}