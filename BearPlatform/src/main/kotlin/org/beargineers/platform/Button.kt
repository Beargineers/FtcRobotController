package org.beargineers.platform

import org.firstinspires.ftc.robotcore.external.Telemetry

open class Button(val test: () -> Boolean) {
    private var pressedAt: Long? = null
    private var onRelease: () -> Unit = {}
    private var onHold: () -> Unit = {}

    fun onRelease(callback: () -> Unit): Button {
        this.onRelease = callback
        return this
    }

    fun onHold(callback: () -> Unit): Button {
        this.onHold = callback
        return this
    }

    open fun update() {
        val pressed = test()
        if (pressed) {
            if (pressedAt == null) {
                pressedAt = System.currentTimeMillis()
            }
            else if (System.currentTimeMillis() - pressedAt!! > 300){
                onHold()
            }
        }
        else if (pressedAt != null) {
            onRelease()
            pressedAt = null
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