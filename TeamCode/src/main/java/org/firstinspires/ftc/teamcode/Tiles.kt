package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

/*
Tiles are marked vertically 1 (lowest row) through 6 (highest row)
and A (leftmost column) through F (rightmost column) horizontally, where A6 holds blue goal and F6 holds red goal.
Each tile is 24x24 inches
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

fun tilePosition(tileCode: String, tileOffset: TileOffset = TileOffset.CENTER): Position2D {
    val (rowCode, columnCode) = tileCode.partition { it.isDigit() }

    val xIn = ('6' - rowCode[0] - 3) * 24 + (tileOffset.xoffset + 1) * 12
    val yIn = (columnCode[0] - 'A' - 3) * 24 + (tileOffset.yoffset + 1) * 12

    return Position2D(xIn.toDouble(), yIn.toDouble(), DistanceUnit.INCH).toUnit(DISTANCE_UNIT)
}
