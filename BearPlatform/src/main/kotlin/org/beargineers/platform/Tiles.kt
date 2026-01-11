package org.beargineers.platform

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

    CENTER_LEFT(-1, 0),
    CENTER_RIGHT(+1, 0),

    TOP_CENTER(0, -1),
    BOTTOM_CENTER(0, +1)
}

fun tileLocation(tileCodeAndOffset: String): Location {
    val tileCode = tileCodeAndOffset.take(2)
    val tileOffset = when(tileCodeAndOffset.drop(2)) {
        "TL" -> TileOffset.TOP_LEFT
        "TR" -> TileOffset.TOP_RIGHT
        "BL" -> TileOffset.BOTTOM_LEFT
        "BR" -> TileOffset.BOTTOM_RIGHT
        "CL" -> TileOffset.CENTER_LEFT
        "CR" -> TileOffset.CENTER_RIGHT
        "TC" -> TileOffset.TOP_CENTER
        "BC" -> TileOffset.BOTTOM_CENTER
        else -> TileOffset.CENTER
    }

    val (rowCode, columnCode) = tileCode.partition { it.isDigit() }

    val xIn = ('6' - rowCode[0] - 3) * 24 + (tileOffset.xoffset + 1) * 12
    val yIn = (columnCode[0] - 'A' - 3) * 24 + (tileOffset.yoffset + 1) * 12

    return Location(xIn.inch, yIn.inch)
}

fun tilePosition(code:String): Position {
    val (tileCodeAndOffset, angle) = code.split(':').map { it.trim() }
    return tileLocation(tileCodeAndOffset).withHeading(angle.toDouble().degrees)
}
