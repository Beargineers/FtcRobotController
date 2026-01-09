package org.beargineers.platform

class PID() {
    private var p: Double = 0.0
    private var i: Double = 0.0
    private var d: Double = 0.0

    private var target = 0.0
    private var error = 0.0
    private var prevError = 0.0
    private var integralError = 0.0
    private var derivativeError = 0.0

    fun setTarget(t: Double, p: Double, i: Double, d: Double) {
        if (t != target || this.p != p || this.i != i || this.d != d) {
            target = t
            this.p = p
            this.i = i
            this.d = d

            error = 0.0
            prevError = 0.0
            integralError = 0.0
            derivativeError = 0.0
        }
    }

    fun update(v: Double) {
        error = target - v
        integralError += error
        derivativeError = error - prevError
        prevError = error
    }

    fun error(): Double = error

    fun result(): Double {
        return p * error + i * integralError + d * derivativeError
    }
}