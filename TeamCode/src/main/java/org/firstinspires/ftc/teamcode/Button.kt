package org.firstinspires.ftc.teamcode

open class Button(val test: () -> Boolean) {
    private var pressed: Boolean = false
    private var callback: () -> Unit = {}

    fun onRelease(callback: () -> Unit): Button {
        this.callback = callback
        return this
    }

    fun update() {
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

class ToggleButton(test: () -> Boolean, val toggleCallback: (Boolean) -> Unit): Button(test) {
    var value: Boolean = false

    init {
        onRelease {
            value = !value
            toggleCallback(value)
        }
    }
}