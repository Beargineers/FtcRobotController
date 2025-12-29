package org.beargineers.platform

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime


abstract class RobotOpMode<T : Robot>(val alliance: Alliance) : OpMode() {
    private val allButtons = mutableListOf<Button>()

    val robot by lazy { RobotFactory.newRobot(this) }
    private val allHubs by lazy {
        hardwareMap.getAll(LynxModule::class.java)
    }

    private val loopsTimer = ElapsedTime()
    private var loopsCount = 0


    private var auto: AutonomousPhase<T>? = null
    private val autoTimer = ElapsedTime()


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

    override fun stop() {
        super.stop()
        robot.stop()
    }

    open fun bearLoop() {}

    private var FPS = 0.0

    final override fun loop() {
        for (hub in allHubs) {
            hub.clearBulkCache()
        }

        telemetry.addData("FPS", "%.1f", FPS)

        loopsCount++
        if (loopsTimer.seconds() > 1) {
            FPS = loopsCount / loopsTimer.seconds()
            loopsCount = 0
            loopsTimer.reset()
        }

        robot.loop()
        allButtons.forEach { it.update() }


        if (controlsAreTouched()) {
            cancelAuto()
        }

        if (auto != null) {
            with(auto!!) {
                if (robot.loopPhase(autoTimer)) {
                    telemetry.addLine("Auto: ${name()}")
                    return
                } else {
                    cancelAuto()
                }
            }
        }

        bearLoop()
    }

    fun button(test: () -> Boolean, callback: () -> Unit) {
        allButtons += Button(test).onRelease(callback)
    }

    fun toggleButton(name: String, test: () -> Boolean, callback: (Boolean) -> Unit) {
        allButtons += ToggleButton(name, telemetry, test, callback)
    }

    fun auto(b: Phases<T>) {
        val builder = PhaseBuilder<T>(this)
        builder.b()
        val phases = builder.build()
        auto = when (phases.size) {
            1 -> phases.first()
            0 -> return
            else -> SequentialPhase("Auto", phases)
        }.also {
            with(it) {
                robot.initPhase()
            }
        }
        autoTimer.reset()
    }

    fun cancelAuto() {
        auto = null
    }

    fun controlsAreTouched(): Boolean {
        return gamepad1.controlsAreTouched() || gamepad2.controlsAreTouched()
    }

    fun Float.touched(): Boolean {
        return this != 0f
    }

    fun Gamepad.controlsAreTouched(): Boolean {
        return left_trigger.touched() || right_trigger.touched() ||
                left_stick_x.touched() || left_stick_y.touched() ||
                right_stick_x.touched() || right_stick_y.touched()
    }
}