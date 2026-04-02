package org.beargineers.platform

import com.bylazar.panels.Panels
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Deferred
import kotlinx.coroutines.Job


abstract class RobotOpMode<T : Robot>() : OpMode() {
    abstract val alliance: Alliance
    val allButtons = mutableListOf<Button>()

    val robot by lazy {
        Config.defaultConfigText(hardwareMap.appContext.resources.openRawResource(RobotFactory.instance.configResource).reader().readText())
        RobotFactory.newRobot(this)
    }

    private val allHubs by lazy {
        hardwareMap.getAll(LynxModule::class.java)
    }


    private var auto: Job? = null
    private val fpsTracker = FPSTracker()

    val elapsedTime = ElapsedTime()
    val loop = LoopRuntime()

    fun isFpsLow(): Boolean = fpsTracker.fpsIsLow

    fun normalTickDurationMs(): Double = fpsTracker.normalTickDurationMs()

    open fun bearInit() {}

    final override fun init() {
        Panels.config.enableLogs = false
        Frame.telemetry = telemetry
        Frame.panelsTelemetry = PanelsTelemetry.telemetry

        elapsedTime.reset()

        // Make sure we read all of the sensor and motor data in one bulk read.
        // This is much faster than reading each of them individually.
        for (hub in allHubs) {
            hub.setBulkCachingMode(BulkCachingMode.MANUAL)
        }

        robot.init()

        bearInit()
        Frame.addLine("Initialized")
    }

    abstract suspend fun T.autoProgram()

    open fun bearStart() {}
    final override fun start() {
        super.start()
        fpsTracker.start()

        robot.start()
        bearStart()
        elapsedTime.reset()

        launch { robot.autoProgram() }.invokeOnCompletion { throwable ->
            if (throwable != null && throwable !is CancellationException) {
                Frame.error(throwable, "Auto program has failed")
                stop()
                terminateOpModeNow()
            }
        }
    }

    override fun stop() {
        super.stop()
        loop.stop()
        robot.stop()
    }

    open fun bearLoop() {}

    final override fun loop() {
        for (hub in allHubs) {
            hub.clearBulkCache()
        }
        fpsTracker.update()

        robot.loop()
        allButtons.forEach { it.update() }

        if (controlsAreTouched()) {
            cancelAuto()
        }

        bearLoop()
        loop.tick()
    }

    fun button(test: () -> Boolean, callback: () -> Unit): Button {
        val button = Button(test).onRelease(callback)
        allButtons += button
        return button
    }

    fun launch(body: suspend CoroutineScope.() -> Unit): Job = async(body)
    fun <T> async(body: suspend CoroutineScope.() -> T): Deferred<T> = loop.submit(body)
    suspend fun yield() {
        loop.nextTick()
    }

    fun auto(name: String, b: suspend T.() -> Unit) {
        cancelAuto()
        auto = launch {
            robot.b()
        }.apply {
            invokeOnCompletion { throwable ->
                if (throwable != null && throwable !is CancellationException) {
                    Frame.error(throwable, "Auto has failed")
                }
            }
        }
    }

    fun cancelAuto() {
        val a = auto
        if (a != null) {
            if (a.isActive) a.cancel()
            auto = null
        }
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

    companion object {
        // This will be used to transfer last known position between different OpModes (like to start Teleop where Auto has finished)
        var lastKnownPosition: Position = Position.zero()
        var lastKnownTurretAngle: Angle = 0.degrees
    }
}