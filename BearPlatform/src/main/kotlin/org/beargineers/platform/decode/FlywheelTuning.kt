package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog

@Disabled
@TeleOp
class FlywheelTuning : OpMode() {
    lateinit var fly1: DcMotorEx
    lateinit var fly2: DcMotorEx
    var maxTicks: Int = 0

    override fun init() {
        fly1 = hardwareMap.get(DcMotorEx::class.java, "fly1")
        fly2 = hardwareMap.get(DcMotorEx::class.java, "fly2")

        fly1.direction = DcMotorSimple.Direction.FORWARD
        fly2.direction = DcMotorSimple.Direction.REVERSE

        fly1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        fly2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        maxTicks = fly1.motorType.achieveableMaxTicksPerSecondRounded
        RobotLog.i("FlyWheelTuning. maxTicks=$maxTicks")
    }

    fun currentTicks(): Int {
        return fly1.velocity.toInt()
    }

    fun currentSpeed(): Double {
        return fly1.velocity / maxTicks
    }

    fun setPower(p: Double) {
        fly1.power = p
        fly2.power = p
    }

    interface Task {
        fun loop(): Boolean
    }

    var task: Task? = null

    override fun loop() {
        if (gamepad1.xWasPressed()) {
            stopTask()
        }

        if (gamepad1.aWasPressed()) {
            task = SpeedUpTest()
        }

        if (gamepad1.bWasPressed()) {
            task = FeedForwardTest()
        }

        if (!(task?.loop() ?: true)) {
            stopTask()
        }
    }

    private fun stopTask() {
        task = null
        setPower(0.0)
    }

    inner class SpeedUpTest(): Task {
        val elapsed = ElapsedTime()
        var logs: Int = 0

        override fun loop(): Boolean {
            setPower(1.0)
            if (elapsed.seconds() > 5) return false

            val ms = elapsed.milliseconds().toInt()
            if (ms >= logs * 50) {
                logs++
                RobotLog.i("Speedup test. Time: $ms. Ticks: ${currentTicks()}, ${currentSpeed()}")
            }

            return true
        }
    }

    inner class FeedForwardTest(): Task {
        var test: Int = 0

        override fun loop(): Boolean {
            if (test++ > 10) return false

            setPower(0.1 * test)
            Thread.sleep(2000)
            RobotLog.i("Power=${0.1 * test}, ticks = ${currentTicks()}")

            return true
        }
    }

}