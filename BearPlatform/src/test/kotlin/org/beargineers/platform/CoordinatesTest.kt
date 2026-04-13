package org.beargineers.platform

import junit.framework.TestCase.assertEquals
import org.beargineers.platform.decode.inShootingZone
import org.junit.Test

class CoordinatesTest : RobotTest() {

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
}