package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position

val DISTANCE_UNIT = DistanceUnit.CM
val ANGLE_UNIT = AngleUnit.DEGREES

/*
Tiles are marked vertically 1 (lowest row) through 6 (highest row)
and A (leftmost column) through F (rightmost column) horizontally, where A6 holds blue goal and F6 holds red goal.
Each tile is 26x26 inches

Make sure to read https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html#square-field-inverted-alliance-area
It says that
- X axis goes from goals towards the audience, i.e. x getting higher when row index getting lower
- Y axis goes from blue goal side towards red goal side, i.e. y getting higher with column index getting higher
- Center of the field has [0,0] coordinates
 */

enum class TileOffset(val yoffset: Int, val xoffset: Int) {
    CENTER(0, 0),

    TOP_LEFT(-1, -1),
    TOP_RIGHT(+1, -1),

    BOTTOM_LEFT(-1, +1),
    BOTTOM_RIGHT(+1, +1),

    LEFT_CENTER(-1, 0),
    RIGHT_CENTER(+1, 0),

    TOP_CENTER(0, -1),
    BOTTOM_CENTER(0, +1)
}

fun tilePosition(tileCode: String, tileOffset: TileOffset = TileOffset.CENTER): Position {
    val (rowCode, columnCode) = tileCode.partition { it.isDigit() }

    val xIn = ('6' - rowCode[0] - 3) * 26 + (tileOffset.xoffset + 1) * 13
    val yIn = (columnCode[0] - 'A' - 3) * 26 + (tileOffset.yoffset + 1) * 13

    return Position(DistanceUnit.INCH, xIn.toDouble(), yIn.toDouble(), 0.0, 0).toUnit(DISTANCE_UNIT)
}

enum class Spike(start: String, end: String) {
    LEFT1("B2", "A2"),
    LEFT2("B3", "A3"),
    LEFT3("B4", "A4"),

    RIGHT1("E2", "F2"),
    RIGHT2("E3", "F3"),
    RIGHT3("E4", "F4");

    val startPosition = tilePosition(start)
    val endPosition = tilePosition(end)
}

val FIELD_CENTER = Position(DISTANCE_UNIT, 0.0, 0.0, 0.0, 0)