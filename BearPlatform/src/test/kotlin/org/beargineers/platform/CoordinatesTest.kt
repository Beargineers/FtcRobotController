package org.beargineers.platform

import junit.framework.TestCase.assertEquals
import junit.framework.TestCase.assertTrue
import org.beargineers.platform.decode.Locations
import org.beargineers.platform.decode.ShootingZones
import org.beargineers.platform.decode.closestPointInShootingZone
import org.beargineers.platform.decode.headingToGoalFrom
import org.beargineers.platform.decode.inShootingZone
import org.beargineers.platform.decode.mirrorForAlliance
import org.beargineers.platform.decode.planShootingApproach
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

    @Test
    fun testClosestPointInShootingZoneRespectsGoalDistanceAndAllianceHalf() {
        robot.assumePosition(Position(30.cm, 25.cm, 180.degrees), 0.degrees)

        val target = robot.closestPointInShootingZone(
            shootingZone = ShootingZones.FRONT,
            stayInAllianceHalf = true
        )
        val targetPose = target.withHeading(robot.headingToGoalFrom(target))
        robot.assumePosition(targetPose, 0.degrees)

        val goal = Locations.GOAL.mirrorForAlliance(robot.alliance)
        val corners = with(robot.getPart(RobotLocations)) {
            listOf(lf_corner, rf_corner, lb_corner, rb_corner)
        }
        val frontZonePenetration = corners.maxOf { -it.x.cm() - it.y.cm() }

        assertTrue(robot.inShootingZone(ShootingZones.FRONT))
        assertTrue(targetPose.distanceTo(goal).cm() >= 80.0)
        assertTrue(corners.all { it.y.cm() >= 0.0 })
        assertTrue(frontZonePenetration >= 5.0)
    }

    @Test
    fun testClosestPointInShootingZoneCanRetreatBackIntoAllianceHalf() {
        robot.assumePosition(Position(-120.cm, 10.cm, 180.degrees), 0.degrees)

        val target = robot.closestPointInShootingZone(
            shootingZone = ShootingZones.FRONT,
            stayInAllianceHalf = true
        )
        val targetPose = target.withHeading(robot.headingToGoalFrom(target))
        robot.assumePosition(targetPose, 0.degrees)

        val corners = with(robot.getPart(RobotLocations)) {
            listOf(lf_corner, rf_corner, lb_corner, rb_corner)
        }

        assertTrue(robot.inShootingZone(ShootingZones.FRONT))
        assertTrue(corners.all { it.y.cm() >= 0.0 })
    }

    @Test
    fun testGammaRedFrontPlannedPoseKeepsMargin() {
        val oldConfig = Config.currentConfigText
        val repoRoot = generateSequence(java.io.File(".").absoluteFile) { it.parentFile }
            .first { java.io.File(it, "settings.gradle").exists() }
        val gammaConfigFile = java.io.File(repoRoot, "Gamma/src/main/res/raw/config.properties")
        try {
            require(gammaConfigFile.exists()) { "Missing Gamma config at ${gammaConfigFile.absolutePath}" }
            Config.updateConfigText(gammaConfigFile.readText())
            robot.hasTurret = true

            robot.assumePosition(Position(52.4.cm, 32.1.cm, (-92.4).degrees), 0.degrees)

            val plan = robot.planShootingApproach(
                shootingZone = ShootingZones.FRONT,
                stayInAllianceHalf = false
            )
            val target = robot.closestPointInShootingZone(
                shootingZone = ShootingZones.FRONT,
                stayInAllianceHalf = false
            )
            val targetPose = plan.target
            robot.assumePosition(targetPose, 0.degrees)

            val corners = with(robot.getPart(RobotLocations)) {
                listOf(lf_corner, rf_corner, lb_corner, rb_corner)
            }
            val penetration = corners.maxOf { -it.x.cm() - kotlin.math.abs(it.y.cm()) }

            assertEquals(plan.target.location(), target)
            assertTrue(robot.inShootingZone(ShootingZones.FRONT))
            assertTrue(penetration >= 5.0)
        } finally {
            robot.hasTurret = false
            Config.updateConfigText(oldConfig)
        }
    }
}