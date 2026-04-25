package org.beargineers.platform

import com.bylazar.panels.Panels
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job

private val PANELS_ENABLED by config(false)

abstract class RobotOpMode<T : Robot> : OpMode() {
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

    enum class PanelsState { ON, OFF, UNKNOWN}
    private var panelsState = PanelsState.UNKNOWN
    private fun checkPanelsState() {
        Panels.config.enableLogs = false

        if (PANELS_ENABLED && panelsState != PanelsState.ON) {
            Panels.config.isDisabled = false
            panelsState = PanelsState.ON
            Panels.enable()
            Frame.panelsTelemetry = PanelsTelemetry.telemetry
        }
        else if (!PANELS_ENABLED && panelsState != PanelsState.OFF && Panels.wasStarted) {
            Panels.config.isDisabled = true
            panelsState = PanelsState.OFF
            Panels.disable()
            Frame.panelsTelemetry = null
        }
    }

    fun isFpsLow(): Boolean = fpsTracker.fpsIsLow

    fun normalTickDurationMs(): Double = fpsTracker.normalTickDurationMs()

    open fun bearInit() {}

    final override fun init() {
        Frame.telemetry = telemetry

        checkPanelsState()

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

        submitJob("Auto Program") { robot.autoProgram() }
    }

    override fun stop() {
        super.stop()
        loop.stop()
        robot.stop()
    }

    open fun bearLoop() {}
    open fun bearInitLoop() {}

    private fun gameModeControls() {
        if (gamepad1.startWasPressed()) {
            setDevMode(false)
        }

        if (gamepad1.backWasPressed()) {
            setDevMode(true)
        }

        Frame.addLine(if (isDevMode()) "DEV MODE" else "GAME MODE")
    }

    final override fun loop() {
        gameModeControls()
        checkPanelsState()

        for (hub in allHubs) {
            hub.clearBulkCache()
        }
        fpsTracker.update()

        robot.loop()
        allButtons.forEach { it.update() }

        bearLoop()
        loop.tick()
    }

    final override fun init_loop() {
        gameModeControls()
        bearInitLoop()
    }

    fun button(test: () -> Boolean, callback: () -> Unit): Button {
        val button = Button(test).onRelease(callback)
        allButtons += button
        return button
    }

    fun submitJob(name: String = "", body: suspend CoroutineScope.() -> Unit): Job {
        if (!name.startsWith("!")) {
            Frame.log("Submitting job: $name")
        }
        return loop.submit(name, body)
    }

    suspend fun nextTick() {
        loop.nextTick()
    }

    fun auto(name: String, b: suspend T.() -> Unit) {
        cancelAuto()
        auto = submitJob(name) {
            robot.b()
        }
    }

    fun cancelAuto() {
        val a = auto
        if (a != null) {
            if (a.isActive) a.cancel()
            auto = null
        }
    }

    fun isAutoActive() = auto?.isActive ?: false

    fun isDevMode(): Boolean {
        return PersistentSettings.getValue("devMode", "false") == "true"
    }

    fun setDevMode(value: Boolean) {
        PersistentSettings.setValue("devMode", if (value) "true" else "false")
    }

    companion object {
        // This will be used to transfer last known position between different OpModes (like to start Teleop where Auto has finished)
        var lastKnownPosition: Position = Position.NA
    }
}