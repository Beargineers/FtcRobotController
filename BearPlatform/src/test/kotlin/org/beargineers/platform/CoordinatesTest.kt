package org.beargineers.platform

import junit.framework.TestCase.assertEquals
import org.beargineers.platform.decode.inShootingZone
import org.junit.Test

class CoordinatesTest : RobotTest() {

    @Test
    fun testInShootingZone() {
        fun test(p: Position, expected: Boolean) {
            robot.assumePosition(p)
            assertEquals(expected, robot.inShootingZone())
        }

        test(Position.zero(), true)
        test(Position.zero().shift(0.cm, 50.cm), false)
        test(Position(180.cm, 0.cm, 0.degrees), true)
    }
}