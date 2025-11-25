package org.beargineers.platform

open class Button<T: BaseRobot>(val robot: T, val test: () -> Boolean) {
    private var pressed: Boolean = false
    private var callback: T.() -> Unit = {}

    fun onRelease(callback: T.() -> Unit): Button<T> {
        this.callback = callback
        return this
    }

    open fun update() {
        val reading = test()
        if (reading) {
            pressed = true
        }
        else if (pressed) {
            pressed = false
            with(robot) {callback()}
        }
    }
}

class ToggleButton<T: BaseRobot>(val name: String, robot: T, test: () -> Boolean, val toggleCallback: T.(Boolean) -> Unit): Button<T>(robot, test) {
    var value: Boolean = false

    init {
        onRelease {
            value = !value
            with(robot) {
                toggleCallback(value)
            }
        }
    }

    override fun update() {
        super.update()
        robot.telemetry.addData(name, if (value) "ON" else "OFF")
    }
}