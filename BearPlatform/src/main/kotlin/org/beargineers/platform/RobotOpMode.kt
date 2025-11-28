package org.beargineers.platform

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime


abstract class RobotOpMode<T: BaseRobot>(val alliance: Alliance) : OpMode() {
    private val allButtons = mutableListOf<Button<T>>()
    protected abstract fun createRobot(opMode: RobotOpMode<T>): T

    val robot by lazy { createRobot(this) }
    private val allHubs by lazy {
        hardwareMap.getAll(LynxModule::class.java)
    }

    private val loopsTimer = ElapsedTime()
    private var loopsCount = 0

    open fun bearInit() {}

    final override fun init() {

        // Make sure we read all of the sensor and motor data in one bulk read.
        // This is much faster than reading each of them individually.
        for (hub in allHubs) {
            hub.setBulkCachingMode(BulkCachingMode.MANUAL)
        }

        robot.init()

        bearInit()
        telemetry.addLine("Initialized")
    }

    open fun bearStart() {}
    final override fun start() {
        super.start()
        loopsTimer.reset()
        loopsCount = 0

        bearStart()
    }

    open fun bearLoop() {}

    private var FPS = 0.0

    final override fun loop() {
        for (hub in allHubs) {
            hub.clearBulkCache()
        }

        telemetry.addData("FPS", "%.1f", FPS)

        if (loopsTimer.seconds() > 1) {
            FPS = loopsCount / loopsTimer.seconds()
            loopsCount = 0
            loopsTimer.reset()
        }

        robot.loop()
        allButtons.forEach { it.update() }

        bearLoop()
        loopsCount++
    }

    fun button(test: () -> Boolean, callback: T.() -> Unit) {
        allButtons += Button(robot, test).onRelease(callback)
    }

    fun toggleButton(name: String, test: () -> Boolean, callback: T.(Boolean) -> Unit) {
        allButtons += ToggleButton(name, robot, test, callback)
    }
}