package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

val FIELD_CENTER = Position2D(0.0, 0.0)

val SOUTH_RED_START_POINT = tilePosition("D1").withHeading(180.0, AngleUnit.DEGREES)
val SOUTH_RED_LAUNCH_POINT = tilePosition("D1").withHeading(160.0, AngleUnit.DEGREES)
val SOUTH_BLUE_START_POINT = tilePosition("C1").withHeading(180.0, AngleUnit.DEGREES)
val SOUTH_BLUE_LAUNCH_POINT = tilePosition("C1").withHeading(-160.0, AngleUnit.DEGREES)

val NORTH_RED_START_POINT = tilePosition("D4").withHeading(90.0, AngleUnit.DEGREES)
val NORTH_RED_LAUNCH_POINT = tilePosition("D4").withHeading(141.0, AngleUnit.DEGREES)
val NORTH_BLUE_LAUNCH_POINT = tilePosition("C4").withHeading(-141.0, AngleUnit.DEGREES)
val NORTH_BLUE_START_POINT = tilePosition("C4").withHeading(-90.0, AngleUnit.DEGREES)

enum class Spike(start: String, end: String) {
    LEFT1("B2", "A2"),
    LEFT2("B3", "A3"),
    LEFT3("B4", "A4"),

    RIGHT1("E2", "F2"),
    RIGHT2("E3", "F3"),
    RIGHT3("E4", "F4");

    val left = name.startsWith("LEFT")
    val heading = if (left) -90.0 else 90.0
    val startPose = tilePosition(start, if (left) TileOffset.RIGHT_CENTER else TileOffset.LEFT_CENTER).withHeading(heading, AngleUnit.DEGREES)
    val endPose = tilePosition(end, if (left) TileOffset.RIGHT_CENTER else TileOffset.LEFT_CENTER).withHeading(heading, AngleUnit.DEGREES)
}
