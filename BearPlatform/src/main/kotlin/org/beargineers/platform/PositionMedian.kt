package org.beargineers.platform

import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.max
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

class PositionMedian(private val minSamples: Int) {
    private val lock = Any()
    private val samples = ArrayDeque<Position>(MAX_SAMPLES)

    fun update(p: Position) {
        synchronized(lock) {
            if (samples.size == MAX_SAMPLES) {
                samples.removeFirst()
            }
            samples.addLast(p.normalizeHeading())
        }
    }

    fun result(
        positionTolerance: Distance,
        headingTolerance: Angle,
        referencePosition: Position? = null
    ): Position? {
        val snapshot = synchronized(lock) { samples.toList() }
        if (snapshot.size < minSamples) return null

        val positionToleranceCm = positionTolerance.cm()
        val headingToleranceDeg = headingTolerance.degrees()
        val reference = referencePosition?.normalizeHeading()

        val candidates = reference
            ?.let { filterNearReference(snapshot, it, positionToleranceCm, headingToleranceDeg) }
            ?.takeIf { it.size >= minSamples }
            ?: snapshot

        val center = robustCenter(candidates, reference)
        val inliers = filterInliers(candidates, center, positionToleranceCm, headingToleranceDeg)

        if (
            inliers.size < minSamples ||
            inliers.size.toDouble() < snapshot.size * MIN_SERIES_INLIER_RATIO
        ) {
            Frame.addData("Vision errors", "rejected %d/%d", inliers.size, snapshot.size)
            return null
        }

        val averaged = average(inliers, center.heading)
        val positionSpread = medianError(inliers) { sample -> translationErrorCm(sample, averaged) }
        val headingSpread = medianError(inliers) { sample -> headingErrorDeg(sample, averaged) }

        if (
            positionSpread > positionToleranceCm * MAX_MEDIAN_SPREAD_RATIO ||
            headingSpread > headingToleranceDeg * MAX_MEDIAN_SPREAD_RATIO
        ) {
            Frame.addData(
                "Vision errors",
                "noisy %.2fcm %.2fdeg %d/%d",
                positionSpread,
                headingSpread,
                inliers.size,
                snapshot.size
            )
            return null
        }

        Frame.addData(
            "Vision errors",
            "%.2fcm %.2fdeg %d/%d",
            positionSpread,
            headingSpread,
            inliers.size,
            snapshot.size
        )

        return averaged
    }

    fun reset() {
        synchronized(lock) {
            samples.clear()
        }
    }

    val n: Int get() = synchronized(lock) { samples.size }

    private fun filterNearReference(
        samples: List<Position>,
        reference: Position,
        positionToleranceCm: Double,
        headingToleranceDeg: Double
    ): List<Position> {
        val positionGate = positionToleranceCm * REFERENCE_GATE_MULTIPLIER
        val headingGate = headingToleranceDeg * REFERENCE_GATE_MULTIPLIER

        return samples.filter { sample ->
            translationErrorCm(sample, reference) <= positionGate &&
                headingErrorDeg(sample, reference) <= headingGate
        }
    }

    private fun robustCenter(samples: List<Position>, reference: Position?): Position {
        val xValues = DoubleArray(samples.size)
        val yValues = DoubleArray(samples.size)
        val headingOffsets = DoubleArray(samples.size)
        val headingAnchor = reference?.heading ?: samples.first().heading

        for (i in samples.indices) {
            val sample = samples[i]
            xValues[i] = sample.x.cm()
            yValues[i] = sample.y.cm()
            headingOffsets[i] = (sample.heading - headingAnchor).normalize().degrees()
        }

        return Position(
            x = median(xValues).cm,
            y = median(yValues).cm,
            heading = (headingAnchor + median(headingOffsets).degrees).normalize()
        )
    }

    private fun filterInliers(
        samples: List<Position>,
        center: Position,
        positionToleranceCm: Double,
        headingToleranceDeg: Double
    ): List<Position> {
        val scoredSamples = ArrayList<ScoredSample>(samples.size)
        val translationErrors = DoubleArray(samples.size)
        val headingErrors = DoubleArray(samples.size)

        for (i in samples.indices) {
            val sample = samples[i]
            val translationError = translationErrorCm(sample, center)
            val headingError = headingErrorDeg(sample, center)
            scoredSamples.add(ScoredSample(sample, translationError, headingError))
            translationErrors[i] = translationError
            headingErrors[i] = headingError
        }

        val translationLimit = robustLimit(
            errors = translationErrors,
            tolerance = positionToleranceCm,
            noiseFloor = MIN_POSITION_NOISE_CM
        )
        val headingLimit = robustLimit(
            errors = headingErrors,
            tolerance = headingToleranceDeg,
            noiseFloor = MIN_HEADING_NOISE_DEG
        )

        return scoredSamples
            .filter { sample ->
                sample.translationErrorCm <= translationLimit &&
                    sample.headingErrorDeg <= headingLimit
            }
            .map { it.sample }
    }

    private fun robustLimit(
        errors: DoubleArray,
        tolerance: Double,
        noiseFloor: Double
    ): Double {
        if (errors.isEmpty()) return tolerance

        val medianError = median(errors.copyOf())
        val deviations = DoubleArray(errors.size)
        for (i in errors.indices) {
            deviations[i] = abs(errors[i] - medianError)
        }
        val sigma = median(deviations) * MAD_TO_SIGMA

        return min(
            tolerance,
            medianError + max(noiseFloor, sigma * OUTLIER_SIGMA_SCALE)
        )
    }

    private fun average(
        samples: List<Position>,
        headingAnchor: Angle
    ): Position {
        var sumX = 0.0
        var sumY = 0.0
        var sumSin = 0.0
        var sumCos = 0.0

        for (sample in samples) {
            sumX += sample.x.cm()
            sumY += sample.y.cm()

            val headingOffset = (sample.heading - headingAnchor).normalize()
            sumSin += kotlin.math.sin(headingOffset.radians())
            sumCos += kotlin.math.cos(headingOffset.radians())
        }

        val averageHeading = if (abs(sumSin) <= EPSILON && abs(sumCos) <= EPSILON) {
            headingAnchor
        } else {
            (headingAnchor + atan2(sumSin, sumCos).radians).normalize()
        }

        return Position(
            x = (sumX / samples.size).cm,
            y = (sumY / samples.size).cm,
            heading = averageHeading
        ).normalizeHeading()
    }

    private fun translationErrorCm(a: Position, b: Position): Double {
        return a.distanceTo(b).cm()
    }

    private fun headingErrorDeg(a: Position, b: Position): Double {
        return abs((a.heading - b.heading).normalize()).degrees()
    }

    private fun median(values: DoubleArray): Double {
        if (values.isEmpty()) return 0.0

        values.sort()
        val mid = values.size / 2
        return if (values.size % 2 == 0) {
            (values[mid - 1] + values[mid]) / 2.0
        } else {
            values[mid]
        }
    }

    private inline fun medianError(samples: List<Position>, error: (Position) -> Double): Double {
        if (samples.isEmpty()) return 0.0

        val errors = DoubleArray(samples.size)
        for (i in samples.indices) {
            errors[i] = error(samples[i])
        }
        return median(errors)
    }

    private data class ScoredSample(
        val sample: Position,
        val translationErrorCm: Double,
        val headingErrorDeg: Double
    )

    private companion object {
        const val MAX_SAMPLES = 20
        const val EPSILON = 1e-9
        const val REFERENCE_GATE_MULTIPLIER = 3.0
        const val MIN_SERIES_INLIER_RATIO = 0.6
        const val OUTLIER_SIGMA_SCALE = 3.0
        const val MAD_TO_SIGMA = 1.4826
        const val MIN_POSITION_NOISE_CM = 0.25
        const val MIN_HEADING_NOISE_DEG = 0.25
        const val MAX_MEDIAN_SPREAD_RATIO = 0.75
    }
}
