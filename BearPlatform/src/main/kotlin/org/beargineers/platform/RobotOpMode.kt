package org.beargineers.platform

import com.qualcomm.robotcore.eventloop.opmode.OpMode

abstract class RobotOpMode<T: BaseRobot>(val alliance: Alliance) : OpMode() {
    private val allButtons = mutableListOf<Button<T>>()
    protected abstract fun createRobot(opMode: RobotOpMode<T>): T

    val robot by lazy { createRobot(this) }

    override fun init() {
        robot.init()
        telemetry.addLine("Initialized")
    }

    override fun loop() {
        robot.loop()
        allButtons.forEach { it.update() }
    }

    fun button(test: () -> Boolean, callback: T.() -> Unit) {
        allButtons += Button(robot, test).onRelease(callback)
    }

    fun toggleButton(name: String, test: () -> Boolean, callback: T.(Boolean) -> Unit) {
        allButtons += ToggleButton(name, robot, test, callback)
    }
}