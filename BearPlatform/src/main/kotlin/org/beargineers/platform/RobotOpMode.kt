package org.beargineers.platform

import com.bylazar.panels.Panels
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime


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


    private var auto: AutonomousPhase<T>? = null
    private val autoTimer = ElapsedTime()
    val elapsedTime = ElapsedTime()


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

    open fun bearStart() {}
    final override fun start() {
        super.start()
        loopTimer.reset()

        bearStart()
        elapsedTime.reset()
    }

    override fun stop() {
        super.stop()
        robot.stop()
    }

    open fun bearLoop() {}

    var lowFPSStartedAt = 0L

    final override fun loop() {
        for (hub in allHubs) {
            hub.clearBulkCache()
        }

        val fps = 1000 / loopTimer.milliseconds()
        loopTimer.reset()
        telemetry.addData("FPS", "%.1f", fps)

        if (fps < 25 && elapsedTime.seconds() > 3) {
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

    fun button(test: () -> Boolean, callback: () -> Unit): Button {
        val button = Button(test).onRelease(callback)
        allButtons += button
        return button
    }

    fun toggleButton(name: String, test: () -> Boolean, callback: (Boolean) -> Unit) {
        allButtons += ToggleButton(name, telemetry, test, callback)
    }

    fun auto(name: String, b: Phases<T>) {
        val builder = PhaseBuilder<T>(this)
        builder.b()
        val phases = builder.build()
        auto = SequentialPhase(name, phases).also {
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

    private var currentFollower: PathFollower? = null

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