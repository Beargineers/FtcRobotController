package org.beargineers.platform

import junit.framework.TestCase.assertNotNull
import junit.framework.TestCase.assertNull
import junit.framework.TestCase.assertTrue
import org.junit.Test
import kotlin.math.abs

class PositionMedianTest {
    @Test
    fun averagesClusterAndRejectsOutliers() {
        val filter = PositionMedian(minSamples = 5)

        listOf(
            Position(100.0.cm, 50.0.cm, 30.0.degrees),
            Position(100.2.cm, 50.1.cm, 29.8.degrees),
            Position(99.8.cm, 49.9.cm, 30.2.degrees),
            Position(100.1.cm, 50.3.cm, 30.1.degrees),
            Position(99.9.cm, 50.0.cm, 29.9.degrees),
            Position(100.0.cm, 49.8.cm, 30.0.degrees),
            Position(130.0.cm, -10.0.cm, -80.0.degrees)
        ).forEach(filter::update)

        val result = filter.result(
            positionTolerance = 2.cm,
            headingTolerance = 2.degrees,
            referencePosition = Position(100.0.cm, 50.0.cm, 30.0.degrees)
        )

        assertNotNull(result)
        assertPoseNear(
            expected = Position(100.0.cm, 50.0.cm, 30.0.degrees),
            actual = result!!,
            positionToleranceCm = 0.35,
            headingToleranceDeg = 0.25
        )
    }

    @Test
    fun rejectsWholeSeriesWhenSamplesSplitAcrossClusters() {
        val filter = PositionMedian(minSamples = 5)

        listOf(
            Position(20.0.cm, 20.0.cm, 10.0.degrees),
            Position(20.2.cm, 19.9.cm, 10.1.degrees),
            Position(19.8.cm, 20.1.cm, 9.8.degrees),
            Position(20.1.cm, 20.0.cm, 10.0.degrees),
            Position(19.9.cm, 20.2.cm, 10.2.degrees),
            Position(40.0.cm, -15.0.cm, -30.0.degrees),
            Position(39.8.cm, -14.9.cm, -30.2.degrees),
            Position(40.2.cm, -15.1.cm, -29.9.degrees),
            Position(40.1.cm, -15.0.cm, -30.1.degrees),
            Position(39.9.cm, -15.2.cm, -30.0.degrees)
        ).forEach(filter::update)

        assertNull(
            filter.result(
                positionTolerance = 3.cm,
                headingTolerance = 3.degrees,
                referencePosition = Position(20.0.cm, 20.0.cm, 10.0.degrees)
            )
        )
    }

    private fun assertPoseNear(
        expected: Position,
        actual: Position,
        positionToleranceCm: Double,
        headingToleranceDeg: Double
    ) {
        assertTrue(abs(expected.x.cm() - actual.x.cm()) <= positionToleranceCm)
        assertTrue(abs(expected.y.cm() - actual.y.cm()) <= positionToleranceCm)
        assertTrue(abs((expected.heading - actual.heading).normalize()).degrees() <= headingToleranceDeg)
    }
}
