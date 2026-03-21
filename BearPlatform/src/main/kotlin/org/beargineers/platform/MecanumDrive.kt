package org.beargineers.platform

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt


class MecanumDrive(robot: BaseRobot) : Hardware(robot) {

    // Match these names in  RC configuration
    val lf: DcMotorEx by hardware("leftFront")
    val rf: DcMotorEx by hardware("rightFront")
    val lb: DcMotorEx by hardware("leftBack")
    val rb: DcMotorEx by hardware("rightBack")

    var ticksPerSecond: Int = 0

    override fun init() {
        val allMotors = listOf(lf, rf, lb, rb)

        lf.direction = WheelsConfig.lf_direction
        lb.direction = WheelsConfig.lb_direction
        rf.direction = WheelsConfig.rf_direction
        rb.direction = WheelsConfig.rb_direction

        ticksPerSecond = lf.motorType.achieveableMaxTicksPerSecondRounded


        allMotors.forEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
    }

    fun drive(forwardPower: Double, rightPower: Double, turnPower: Double) {
        val strafe = rightPower * sqrt(2.0) // strafe is less effective than forward movement with same power applied

        val lfP = forwardPower + strafe + turnPower
        val rfP = forwardPower - strafe - turnPower
        val lbP = forwardPower - strafe + turnPower
        val rbP = forwardPower + strafe - turnPower

        val maxMag = listOf(1.0, lfP, rfP, lbP, rbP).maxOf { abs(it) }

        val limit = WheelsConfig.topSpeed
        fun normalize(v: Double) = roundMotorPower((v / maxMag) * limit) * ticksPerSecond

        lf.velocity = normalize(lfP * WheelsConfig.lf_correction)
        rf.velocity = normalize(rfP * WheelsConfig.rf_correction)
        lb.velocity = normalize(lbP * WheelsConfig.lb_correction)
        rb.velocity = normalize(rbP * WheelsConfig.rb_correction)
    }

    fun driveByPowerAndAngle(theta: Double, power: Double, turn: Double) {
        val power = power * WheelsConfig.topSpeed
        val turn = turn * WheelsConfig.topSpeed

        val sin = sin(theta + Math.PI / 4)
        val cos = cos(theta + Math.PI / 4)
        val max = max(abs(sin), abs(cos))

        val scale = max(1.0, abs(power) + abs(turn))

        val lfP = (power * cos / max + turn) / scale
        val rfP = (power * sin / max - turn) / scale
        val lbp = (power * sin / max + turn) / scale
        val rbp = (power * cos / max - turn) / scale

        fun normalize(v: Double) = roundMotorPower(v) * ticksPerSecond
        lf.velocity = normalize(lfP * WheelsConfig.lf_correction)
        rf.velocity = normalize(rfP * WheelsConfig.rf_correction)
        lb.velocity = normalize(lbp * WheelsConfig.lb_correction)
        rb.velocity = normalize(rbp * WheelsConfig.rb_correction)
    }

    override fun stop() {
        lf.power = 0.0
        rf.power = 0.0
        lb.power = 0.0
        rb.power = 0.0
    }
}

object WheelsConfig {
    val RoadRunnerEnabled by config(false)
    val topSpeed by config(1.0)
    val lf_direction by config(DcMotorSimple.Direction.REVERSE)
    val rf_direction by config(DcMotorSimple.Direction.FORWARD)
    val lb_direction by config( DcMotorSimple.Direction.REVERSE)
    val rb_direction by config(DcMotorSimple.Direction.FORWARD)

    val lf_correction by config(1.0)
    val rf_correction by config(1.0)
    val lb_correction by config(1.0)
    val rb_correction by config(1.0)
}
