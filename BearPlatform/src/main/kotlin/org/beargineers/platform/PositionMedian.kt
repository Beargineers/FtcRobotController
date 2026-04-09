package org.beargineers.platform

import kotlin.math.abs
import kotlin.math.min

class DoubleMedian(val maxSamples:Int = 20) {
    private val data = DoubleArray(maxSamples)

    private val deviations = DoubleArray(maxSamples)
    private val meanSamples = mutableListOf<Double>()
    var n: Int = 0

    fun update(d: Double) {
        data[n++ % maxSamples] = d
    }

    fun median(): Double {
        if (n == 0) return 0.0
        return data.median(min(n, maxSamples))
    }

    fun mean(): Double {
        if (n == 0) return 0.0
        val median = median()
        val sigma = mad() * 1.4826
        val size = min(n, maxSamples)

        meanSamples.clear()
        for (i in 0 until size) {
            if (abs(data[i] - median) <= 3 * sigma) {
                meanSamples.add(data[i])
            }
        }

        if (meanSamples.isEmpty()) return 0.0

        val goodSamplesSize = min(5, meanSamples.size)
        meanSamples.sortBy { abs(it - median) }

        return meanSamples.subList(0, goodSamplesSize).average()
    }

    private fun DoubleArray.median(n: Int): Double {
        sort(0, n)
        val mid = n / 2

        return if (n % 2 == 0) {
            (this[mid - 1] + this[mid]) / 2.0
        } else {
            this[mid]
        }
    }

    fun mad(): Double {
        if (n == 0) return 0.0

        val med = median()
        val size = min(n, maxSamples)
        for (i in 0 until size) {
            deviations[i] = abs(data[i] - med)
        }

        return deviations.median(size)
    }

    fun result(): Pair<Double, Double> {
        return mean() to mad()
    }

    fun reset() {
        n = 0
    }
}

class PositionMedian(val minSamples: Int) {
    private val x = DoubleMedian()
    private val y = DoubleMedian()
    private val heading = DoubleMedian()

    fun update(p: Position) {
        x.update(p.x.cm())
        y.update(p.y.cm())
        heading.update(p.heading.degrees())
    }

    fun result(positionTolerance: Distance, headingTolerance: Angle): Position? {
        if (n < minSamples) return null

        val (xm, xstd) = x.result()
        val (ym, ystd) = y.result()
        val (hm, hstd) = heading.result()

        Frame.addData("Vision errors", "%.2f %.2f %.2f", xstd, ystd, hstd)

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