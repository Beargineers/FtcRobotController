package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.internal.Hardware

private const val FEEDER_RUN_TIME: Long = 3500L

@Configurable
object ShooterConfig {
    var SHOOTER_P = 0.65
}

class Shooter(op: OpMode): Hardware(op) {
    val fly1 by hardware<DcMotor>()
    val fly2 by hardware<DcMotor>()

    val feeder by hardware<DcMotor>()

    var feederStartedAt: Long = 0

    val launchMotors = listOf(fly1, fly2)

    init {
        launchMotors.forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        feeder.direction = DcMotorSimple.Direction.REVERSE
        feeder.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    fun enableFlywheel(on: Boolean) {
        if (on) {
            powerFlywheel(ShooterConfig.SHOOTER_P)
        }
        else {
            powerFlywheel(0.0)
        }
    }

    private fun powerFlywheel(p: Double) {
        fly1.direction = DcMotorSimple.Direction.REVERSE
        fly2.direction = DcMotorSimple.Direction.REVERSE
        setMotorPower(fly1, p)
        setMotorPower(fly2, p)
    }

    fun launch() {
        feeder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        setMotorPower(feeder, 1.0)
        feederStartedAt = System.currentTimeMillis()
    }

    override fun loop() {
        if (feederStartedAt != 0L && (System.currentTimeMillis() - feederStartedAt) > FEEDER_RUN_TIME) {
            setMotorPower(feeder, 0.0)
            feederStartedAt = 0L
        }
    }

    override fun stop() {
        enableFlywheel(false)
        setMotorPower(feeder, 0.0)
    }
}