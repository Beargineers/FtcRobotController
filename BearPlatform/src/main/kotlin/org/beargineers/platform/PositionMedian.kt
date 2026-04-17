package org.beargineers.platform

import kotlin.math.abs
import kotlin.math.atan2
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
    private val lock = Any()
    private val samples = ArrayDeque<Position>(MAX_SAMPLES)

    fun update(p: Position) {
        synchronized(lock) {
            if (samples.size == MAX_SAMPLES) {
                samples.removeAt(0)
            }
            samples.add(p.normalizeHeading())
        }
    }

    fun result(positionTolerance: Distance, headingTolerance: Angle): Position? {
        val snapshot = synchronized(lock) { samples.toList() }
        if (snapshot.size < minSamples) return null

        val consensus = bestConsensus(snapshot, positionTolerance, headingTolerance) ?: return null
        if (consensus.samples.size < snapshot.size * 0.75) return null // Require 3/4 good samples

        val result = weightedAverage(consensus.center, consensus.samples, positionTolerance, headingTolerance)
        val positionSpread = median(consensus.samples.map { translationErrorCm(it, result) })
        val headingSpread = median(consensus.samples.map { headingErrorDeg(it, result) })

        Frame.addData(
            "Vision errors",
            "%.2fcm %.2fdeg %d/%d",
            positionSpread,
            headingSpread,
            consensus.samples.size,
            snapshot.size
        )

        return result
    }

    fun reset() {
        synchronized(lock) {
            samples.clear()
        }
    }

    val n: Int get() = synchronized(lock) { samples.size }

    private fun bestConsensus(
        samples: List<Position>,
        positionTolerance: Distance,
        headingTolerance: Angle
    ): PoseConsensus? {
        val positionToleranceCm = positionTolerance.cm()
        val headingToleranceDeg = headingTolerance.degrees()

        var best: PoseConsensus? = null
        var bestScore = Double.POSITIVE_INFINITY

        for (center in samples) {
            val consensus = PoseConsensus(
                center = center,
                samples = samples.filter { sample -> isInTolerance(sample, center, positionToleranceCm, headingToleranceDeg) }
            )
            val score = consensus.score(positionToleranceCm, headingToleranceDeg)

            if (
                best == null ||
                consensus.samples.size > best.samples.size ||
                (consensus.samples.size == best.samples.size && score < bestScore)
            ) {
                best = consensus
                bestScore = score
            }
        }

        return best
    }

    private fun isInTolerance(
        sample: Position,
        center: Position,
        positionToleranceCm: Double,
        headingToleranceDeg: Double
    ): Boolean {
        return translationErrorCm(sample, center) <= positionToleranceCm &&
            headingErrorDeg(sample, center) <= headingToleranceDeg
    }

    private fun weightedAverage(
        center: Position,
        clusteredSamples: List<Position>,
        positionTolerance: Distance,
        headingTolerance: Angle
    ): Position {
        val positionToleranceCm = positionTolerance.cm().coerceAtLeast(EPSILON)
        val headingToleranceDeg = headingTolerance.degrees().coerceAtLeast(EPSILON)

        var sumX = 0.0
        var sumY = 0.0
        var sumSin = 0.0
        var sumCos = 0.0
        var totalWeight = 0.0

        for (sample in clusteredSamples) {
            val positionRatio = translationErrorCm(sample, center) / positionToleranceCm
            val headingRatio = headingErrorDeg(sample, center) / headingToleranceDeg
            val weight = 1.0 / (1.0 + positionRatio * positionRatio + headingRatio * headingRatio)

            sumX += sample.x.cm() * weight
            sumY += sample.y.cm() * weight
            sumSin += kotlin.math.sin(sample.heading.radians()) * weight
            sumCos += kotlin.math.cos(sample.heading.radians()) * weight
            totalWeight += weight
        }

        if (totalWeight <= EPSILON) return center.normalizeHeading()

        val averageHeading = if (abs(sumSin) <= EPSILON && abs(sumCos) <= EPSILON) {
            center.heading
        } else {
            atan2(sumSin, sumCos).radians
        }

        return Position(
            x = (sumX / totalWeight).cm,
            y = (sumY / totalWeight).cm,
            heading = averageHeading
        ).normalizeHeading()
    }

    private fun translationErrorCm(a: Position, b: Position): Double {
        return a.distanceTo(b).cm()
    }

    private fun headingErrorDeg(a: Position, b: Position): Double {
        return abs((a.heading - b.heading).normalize()).degrees()
    }

    private fun median(values: List<Double>): Double {
        if (values.isEmpty()) return 0.0

        val sorted = values.sorted()
        val mid = sorted.size / 2
        return if (sorted.size % 2 == 0) {
            (sorted[mid - 1] + sorted[mid]) / 2.0
        } else {
            sorted[mid]
        }
    }

    private data class PoseConsensus(
        val center: Position,
        val samples: List<Position>
    ) {
        fun score(positionToleranceCm: Double, headingToleranceDeg: Double): Double {
            val safePositionTolerance = positionToleranceCm.coerceAtLeast(EPSILON)
            val safeHeadingTolerance = headingToleranceDeg.coerceAtLeast(EPSILON)

            return samples.sumOf { sample ->
                val positionRatio = sample.distanceTo(center).cm() / safePositionTolerance
                val headingRatio = abs((sample.heading - center.heading).normalize()).degrees() / safeHeadingTolerance
                positionRatio * positionRatio + headingRatio * headingRatio
            }
        }
    }

    private companion object {
        const val MAX_SAMPLES = 20
        const val EPSILON = 1e-9
    }
}
