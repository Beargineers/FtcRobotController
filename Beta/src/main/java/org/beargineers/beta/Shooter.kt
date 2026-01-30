package org.beargineers.beta

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Hardware
import org.beargineers.platform.PID
import org.beargineers.platform.PIDFTCoeffs
import org.beargineers.platform.config
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.platform.decode.goalDistance
import org.beargineers.platform.roundMotorPower
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.abs

class Shooter(robot: BaseRobot): Hardware(robot) {
    val SHOOTER_POWER_ADJUST by config(1.0)
    var manualPowerAdjustment = 1.0
    val SHOOTER_DISTANCE_QUOTIENT by config(0.00101)
    val SHOOTER_FREE_QUOTIENT by config(0.556)
    val SHOOTING_TIME_SECONDS by config(4.5)
    val SHOOTER_ERROR_MARGIN by config(0.03)

    val SHOOTER_PID by config(PIDFTCoeffs(0.0, 0.0, 0.0, 0.0))
    val SHOOTER_ANGLE_CORRECTION by config(0.0)

    val pid = PID()

    val fly1 by hardware<DcMotor>()
    val fly2 by hardware<DcMotor>()

    val feeder by hardware<DcMotor>()

    val shooterBallDetector by hardware<DistanceSensor>()

    val shooterBallDetectorThreshold by config(15.0)
    val stopFeederDelay by config(100)

    var feederTransferring = false
    var stopFeederAt: Long = 0

    var flywheelEnabled = false

    var feederShooting = false

    var maxTicks = 0

    val loopTime = ElapsedTime()


    override fun init() {
        super.init()

        val launchMotors = listOf(fly1, fly2)
        fly1.direction = DcMotorSimple.Direction.REVERSE
        fly2.direction = DcMotorSimple.Direction.FORWARD
        launchMotors.forEach {
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        }

        feeder.direction = DcMotorSimple.Direction.REVERSE
        feeder.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        maxTicks = fly1.motorType.achieveableMaxTicksPerSecondRounded
    }

    fun enableFlywheel(on: Boolean) {
        flywheelEnabled = on
    }

    private fun powerFlywheel(p: Double) {
        pid.updateCoefficients(SHOOTER_PID)
        pid.setTarget(p)
        pid.updateCurrent((fly1 as DcMotorEx).velocity / (maxTicks))
        telemetry.addData("Shooter error", pid.error())
        robot.panelsTelemetry.addData("Shooter error", pid.error())

        val v = roundMotorPower(pid.result() + p)
        fly1.power = v
        fly2.power = v
    }

    fun getReadyForShoot(){
        if (!feederShooting) {
            feederTransferring = true
        }
    }
    fun launch() {
        feeder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        feederShooting = true
        stopFeederAt = System.currentTimeMillis() + (SHOOTING_TIME_SECONDS * 1000).toLong()
        (robot as BetaRobot).intake.onShoot()
    }

    private fun recommendedFlywheelPower(): Double = flywheelPowerAdjustedToDistance((this@Shooter.robot as DecodeRobot).goalDistance().cm())

    override fun loop() {
        val dt = loopTime.milliseconds()
        val now = System.currentTimeMillis()

        loopTime.reset()

        if (feederShooting || feederTransferring) {
            val ballDistance = shooterBallDetector.getDistance(DistanceUnit.CM)
            if(feederTransferring && ballDistance < shooterBallDetectorThreshold){
                feederTransferring = false
            }
            if (feederShooting && ballDistance < shooterBallDetectorThreshold) {
                stopFeederAt = now + stopFeederDelay
            }
        }

        if (feederShooting && now >= stopFeederAt) {
            feederShooting = false
            stopFeederAt = 0L
        }

        val flywheelPoweredUp = abs(pid.error()) < SHOOTER_ERROR_MARGIN
        if (feederShooting && !flywheelPoweredUp) {
            stopFeederAt += dt.toLong()
        }

        feeder.power = if (feederShooting && flywheelPoweredUp && flywheelEnabled || feederTransferring) 1.0 else 0.0

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
        feeder.power = 0.0
    }
}