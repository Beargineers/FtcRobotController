package org.beargineers.platform

import kotlin.math.min
import kotlin.math.sqrt

private val N = 300

private class DoubleNormalDistribution {
    private val data = DoubleArray(N)
    var n: Int = 0

    fun update(d: Double) {
        data[n++ % N] = d
    }

    fun result(): Pair<Double, Double> {
        var mean = 0.0
        var m2 = 0.0
        var delta: Double
        var delta2: Double

        val last = min(n, N) - 1

        for (i in 0..last) {
            val d = data[i]
            delta = d - mean
            mean += delta / (i + 1)
            delta2 = d - mean
            m2 += delta * delta2
        }

        val stdDev = if (last <= 0) 0.0 else sqrt(m2 / last)
        return Pair(mean, stdDev)
    }

    fun reset() {
        n = 0
    }
}

class PositionNormalDistribution(val positionTolerance: Distance, val headingTolerance: Angle, val minSamples: Int) {
    private val x = DoubleNormalDistribution()
    private val y = DoubleNormalDistribution()
    private val heading = DoubleNormalDistribution()

    fun update(p: Position) {
        x.update(p.x.cm())
        y.update(p.y.cm())
        heading.update(p.heading.degrees())
    }

    fun result(): Position? {
        if (n < minSamples) return null

        val (xm, xstd) = x.result()
        val (ym, ystd) = y.result()
        val (hm, hstd) = heading.result()

        if (xstd > positionTolerance.cm() || ystd > positionTolerance.cm() || hstd > headingTolerance.degrees()) return null

        return Position(xm.cm, ym.cm, hm.degrees)
    }

    fun reset() {
        x.reset()
        y.reset()
        heading.reset()
    }

    val n: Int get() = x.n
}