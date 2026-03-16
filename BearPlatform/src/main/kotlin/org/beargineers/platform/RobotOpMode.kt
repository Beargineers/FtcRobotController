package org.beargineers.platform

import com.bylazar.panels.Panels
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.CompletableDeferred


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

    private val loopTimer = ElapsedTime()


    private var auto: CompletableDeferred<Unit>? = null
    val elapsedTime = ElapsedTime()
    val loop = LoopRuntime()


    open fun bearInit() {}

    final override fun init() {
        Panels.config.enableLogs = false

        elapsedTime.reset()

        // Make sure we read all of the sensor and motor data in one bulk read.
        // This is much faster than reading each of them individually.
        for (hub in allHubs) {
            hub.setBulkCachingMode(BulkCachingMode.MANUAL)
        }

        robot.init()

        bearInit()
        telemetry.addLine("Initialized")
    }

    abstract suspend fun T.autoProgram()

    open fun bearStart() {}
    final override fun start() {
        super.start()
        loopTimer.reset()

        bearStart()
        loop.submit { robot.autoProgram() }

        elapsedTime.reset()
    }

    override fun stop() {
        super.stop()
        loop.stop()
        robot.stop()
    }

    open fun bearLoop() {}

    var lowFPSStartedAt = 0L

    val fpsDist = DoubleNormalDistribution(100)

    final override fun loop() {
        for (hub in allHubs) {
            hub.clearBulkCache()
        }

        fpsDist.update(1000 / loopTimer.milliseconds())
        loopTimer.reset()
        val (fps, std) = fpsDist.result()
        telemetry.addData("FPS", "%.1f, STD=%.1f", fps, std)

        if (fps < 20 && elapsedTime.seconds() > 3) {
            val now = System.currentTimeMillis()
            if (lowFPSStartedAt == 0L) {
                lowFPSStartedAt = now
            }

            if (now - lowFPSStartedAt > 500) {
                robot.lowFpsMode(true)
            }
        }
        else {
            lowFPSStartedAt = 0L
        }

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

    fun toggleButton(name: String, test: () -> Boolean, callback: (Boolean) -> Unit) {
        allButtons += ToggleButton(name, telemetry, test, callback)
    }

    fun auto(name: String, b: suspend T.() -> Unit) {
        cancelAuto()
        auto = loop.submit {
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

    private var currentFollower: PathFollower? = null

    // TODO: Remove when RR implementation is secured
    fun followPath(waypoints: List<Waypoint>): Boolean {
        // Check if this is a new path (different instance)
        if (currentFollower?.path != waypoints) {
            // Create new PathFollower for this path
            currentFollower = PathFollower(
                robot = robot as BaseRobot,
                path = waypoints,
                startPosition = robot.currentPosition
            )
        }

        return currentFollower!!.update().also {
            if (!it) currentFollower = null
        }
    }

    companion object {
        // This will be used to transfer last known position between different OpModes (like to start Teleop where Auto has finished)
        var lastKnownPosition: Position = Position.zero()
    }
}