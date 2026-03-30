package org.beargineers.platform

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
            else if (System.currentTimeMillis() - pressedAt!! > 200){
                onHold()
            }
        }
        else if (pressedAt != null) {
            onRelease()
            pressedAt = null
        }
    }
}
