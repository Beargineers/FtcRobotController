@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.internal

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.auto
import org.firstinspires.ftc.teamcode.teleop


abstract class RobotOpMode() : OpMode() {
    lateinit var robot: Robot
    val elapsed = ElapsedTime()

    override fun init() {
        robot = Robot(this)
    }

    override fun start() {
        elapsed.reset()
    }

    inline fun <reified H> hardware(name: String): H {
        return hardwareMap.get(H::class.java, name)
    }
}


@Autonomous(name = "Auto mode", group = "Auto")
class AutoMode() : RobotOpMode() {
    override fun loop() {
        auto()
    }
}

@TeleOp(name = "Teleop mode", group = "Teleop")
class TeleOpMode() : RobotOpMode() {
    override fun loop() {
        teleop()
    }
}
