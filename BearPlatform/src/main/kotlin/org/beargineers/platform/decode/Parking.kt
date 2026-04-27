package org.beargineers.platform.decode

import org.beargineers.platform.Alliance
import org.beargineers.platform.Angle
import org.beargineers.platform.Location
import org.beargineers.platform.Position
import org.beargineers.platform.RobotDimensions
import org.beargineers.platform.abs
import org.beargineers.platform.cm
import org.beargineers.platform.config
import org.beargineers.platform.degrees
import org.beargineers.platform.driveTo
import org.beargineers.platform.inch
import org.beargineers.platform.tileLocation
import org.beargineers.platform.withName

val PARKING_SAFETY_MARGIN by config(0.cm)

fun DecodeRobot.parkingSpotCenter(): Location {
    return parkingSpotCenterUnmirrored().mirrorForAlliance(alliance)
}

private fun parkingSpotCenterUnmirrored(): Location {
    return tileLocation("B2BR").shift(-9.inch, -9.inch)
}

enum class ParkingSide {
    UP, DOWN, LEFT, RIGHT
}

suspend fun DecodeRobot.fullPark(wall: Boolean) {
    withName("Full Parking. wall=$wall") {
        val safetyMargin = PARKING_SAFETY_MARGIN
        val boxSize = 18.inch - safetyMargin * 2
        val brCorner = tileLocation("B2BR").shift(-safetyMargin, -safetyMargin)
        val trCorner = brCorner.shift(-boxSize, 0.inch)

        val blCorner = brCorner.shift(0.inch, -boxSize)
        val tlCorner = blCorner.shift(boxSize, 0.inch)

        val width = RobotDimensions.ROBOT_WHEELBASE_WIDTH
        val half = width / 2

        fun Location.toBestPosition(vararg angles: Angle): Position {
            fun Angle.mirrorForAlliance() = if (alliance == Alliance.RED) this else -this
            val heading = currentPosition.heading
            return mirrorForAlliance(alliance).withHeading(angles.minBy { abs((it.mirrorForAlliance() - heading).normalize()) }.mirrorForAlliance())
        }

        val wallParking = blCorner.shift(-half, half).toBestPosition(-90.degrees, 0.degrees)
        val fieldParking = brCorner.shift(-half, -half).toBestPosition(0.degrees, 90.degrees)

        driveTo(if (wall) wallParking else fieldParking, applyMirroring = false)
    }
}

suspend fun DecodeRobot.partialPark(side: ParkingSide) {
    withName("Partial parking: $side") {
        val center = parkingSpotCenterUnmirrored()

        val width = RobotDimensions.ROBOT_WHEELBASE_WIDTH
        val half = width / 2

        val shift = 9.inch + half - 2.inch

        val spot = when (side) {
            ParkingSide.UP -> center.shift(0.cm, shift)
            ParkingSide.DOWN -> center.shift(0.cm, -shift)
            ParkingSide.LEFT -> center.shift(-shift * alliance.sign, 0.cm)
            ParkingSide.RIGHT -> center.shift(shift * alliance.sign, 0.cm)
        }

        val angles = when (side) {
            ParkingSide.UP,
            ParkingSide.DOWN -> listOf(0.degrees, 180.degrees)
            ParkingSide.LEFT,
            ParkingSide.RIGHT -> listOf(90.degrees, -90.degrees)
        }

        val heading = currentPosition.heading
        val parking = spot.mirrorForAlliance(alliance).withHeading(
            angles.minBy { abs((it - heading).normalize()) }
        )

        driveTo(parking, applyMirroring = false)
    }
}
