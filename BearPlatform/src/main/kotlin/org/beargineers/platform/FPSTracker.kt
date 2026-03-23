package org.beargineers.platform

import com.qualcomm.robotcore.util.ElapsedTime

class FPSTracker {
    private val loopTimer = ElapsedTime()
    private val elapsedTime = ElapsedTime()
    private var lowFPSStartedAt = 0L
    private var normalFPSStartedAt = 0L

    var fpsIsLow: Boolean = false

    private val fpsDist = DoubleNormalDistribution(100)

    fun start() {
        loopTimer.reset()
        elapsedTime.reset()
        fpsIsLow = false
        lowFPSStartedAt = 0L
        normalFPSStartedAt = 0L
        fpsDist.reset()
    }

    fun update() {
        fpsDist.update(1000 / loopTimer.milliseconds())
        loopTimer.reset()
        val (fps, std) = fpsDist.result()

        if (elapsedTime.seconds() > 3) { // Give JIT time to speed up the loop
            val now = System.currentTimeMillis()

            if (fps < 20) {
                normalFPSStartedAt = 0L
                if (lowFPSStartedAt == 0L) lowFPSStartedAt = now
                if (now - lowFPSStartedAt > 500) fpsIsLow = true
            } else if (fps > 30) {
                lowFPSStartedAt = 0L
                if (normalFPSStartedAt == 0L) normalFPSStartedAt = now
                if (now - normalFPSStartedAt > 800) fpsIsLow = false
            } else {
                lowFPSStartedAt = 0L
                normalFPSStartedAt = 0L
            }
        }

        Frame.addData("FPS", "${if (fpsIsLow) "LOW!!" else ""} %.1f, STD=%.1f", fps, std)
    }
}