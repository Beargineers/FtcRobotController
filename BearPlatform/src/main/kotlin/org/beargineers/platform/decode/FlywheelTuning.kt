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
    lateinit var intake: DcMotorEx
    var maxTicks: Int = 0

    override fun init() {
        fly1 = hardwareMap.get(DcMotorEx::class.java, "fly1")
        fly2 = hardwareMap.get(DcMotorEx::class.java, "fly2")
        intake = hardwareMap.get(DcMotorEx::class.java, "intake")


        fly1.direction = DcMotorSimple.Direction.FORWARD
        fly2.direction = DcMotorSimple.Direction.REVERSE
        intake.direction = DcMotorSimple.Direction.REVERSE


        fly1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        fly2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

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

        if (gamepad1.yWasPressed()) {
            intake.power = 1.0
            task = ShootTest()
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

    inner class ShootTest() : Task {
        val dip = ElapsedTime()
        var speedingUp = true
        var lastTicks = 0
        var maxTicks = 0
        var minTicks = 0
        var counter = 0

        var testN = 0

        var p = 0.3

        override fun loop(): Boolean {
            setPower(p)

            val curTicks = currentTicks()

            if (speedingUp) {
                if (lastTicks - curTicks > 20) {
                    speedingUp = false
                    dip.reset()
                }
                else {
                    maxTicks = curTicks
                }
            }
            else {
                if (curTicks > lastTicks) {
                    speedingUp = true
                    RobotLog.i("Power $p. #${counter+1} Drop ${maxTicks - minTicks} from $maxTicks. Recovered in ${dip.milliseconds()}ms")

                    if (counter++ >= 5) {
                        testN++
                        counter = 0
                        p += 0.2
                    }
                }
                else {
                    minTicks = curTicks
                }
            }

            lastTicks = curTicks

            return testN < 3
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