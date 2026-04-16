package org.beargineers.platform

import junit.framework.TestCase.assertEquals
import junit.framework.TestCase.assertTrue
import org.beargineers.platform.decode.inShootingZone
import org.junit.Test

class CoordinatesTest : RobotTest() {
    private fun assertDistanceEquals(
        expected: Distance,
        actual: Distance,
        toleranceCm: Double = 1e-6
    ) {
        assertTrue(
            "expected=$expected actual=$actual",
            kotlin.math.abs(expected.cm() - actual.cm()) <= toleranceCm
        )
    }

    private fun assertLocationEquals(
        expected: Location,
        actual: Location,
        toleranceCm: Double = 1e-6
    ) {
        assertDistanceEquals(expected.x, actual.x, toleranceCm)
        assertDistanceEquals(expected.y, actual.y, toleranceCm)
    }

    private fun assertRobotLocationEquals(
        expected: RobotCentricLocation,
        actual: RobotCentricLocation,
        toleranceCm: Double = 1e-6
    ) {
        assertDistanceEquals(expected.forward, actual.forward, toleranceCm)
        assertDistanceEquals(expected.right, actual.right, toleranceCm)
    }

    @Test
    fun testInShootingZone() {
        fun test(p: Position, expected: Boolean) {
            robot.assumePosition(p, 0.degrees)
            assertEquals(expected, robot.inShootingZone())
        }

        test(Position.zero(), true)
        test(Position.zero().shift(0.cm, 50.cm), false)
        test(Position(180.cm, 0.cm, 0.degrees), true)
    }


    @Test
    fun testHeadinToFrom() {
        assertEquals(0.degrees, headingFromTo(Position.zero().location(), Location(10.cm, 0.cm)))
        assertEquals(90.degrees, headingFromTo(Position.zero().location(), Location(0.cm, 10.cm)))
        assertEquals(180.degrees, headingFromTo(Position.zero().location(), Location(-10.cm, 0.cm)))
    }

    @Test
    fun testToFieldCentricCardinalDirections() {
        assertLocationEquals(
            Location(10.cm, 0.cm),
            RobotCentricLocation(10.cm, 0.cm).toFieldCentric(Position.zero())
        )
        assertLocationEquals(
            Location(0.cm, -10.cm),
            RobotCentricLocation(0.cm, 10.cm).toFieldCentric(Position.zero())
        )
        assertLocationEquals(
            Location(0.cm, 10.cm),
            RobotCentricLocation(10.cm, 0.cm).toFieldCentric(Position(0.cm, 0.cm, 90.degrees))
        )
        assertLocationEquals(
            Location(10.cm, 0.cm),
            RobotCentricLocation(0.cm, 10.cm).toFieldCentric(Position(0.cm, 0.cm, 90.degrees))
        )
    }

    @Test
    fun testFieldRobotCentricTransformsAreInverse() {
        val start = Position(25.cm, -40.cm, 37.degrees)
        val robotCentric = RobotCentricLocation(18.cm, 7.cm)

        val fieldLocation = robotCentric.toFieldCentric(start)
        val roundTrip = fieldLocation.toRobotCentric(start)

        assertRobotLocationEquals(robotCentric, roundTrip)
    }
}