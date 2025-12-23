package org.beargineers.platform

import org.firstinspires.ftc.robotcore.external.Telemetry

open class Button(val test: () -> Boolean) {
    private var pressed: Boolean = false
    private var callback: () -> Unit = {}

    fun onRelease(callback: () -> Unit): Button {
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
            callback()
        }
    }
}

class ToggleButton(val name: String, val telemetry: Telemetry, test: () -> Boolean, val toggleCallback: (Boolean) -> Unit): Button(test) {
    var value: Boolean = false

    init {
        onRelease {
            value = !value
            toggleCallback(value)
        }
    }

    override fun update() {
        super.update()
        telemetry.addData(name, if (value) "ON" else "OFF")
    }
}