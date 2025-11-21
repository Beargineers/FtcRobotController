package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

enum class Spike(start: String, end: String) {
    LEFT1("B2", "A2"),
    LEFT2("B3", "A3"),
    LEFT3("B4", "A4"),

    RIGHT1("E2", "F2"),
    RIGHT2("E3", "F3"),
    RIGHT3("E4", "F4");

    val left = name.startsWith("LEFT")
    val heading = if (left) -90.0 else 90.0
    val startPose = tilePosition(start, if (left) TileOffset.CENTER_RIGHT else TileOffset.CENTER_LEFT).withHeading(heading, AngleUnit.DEGREES)
    val endPose = tilePosition(end, if (left) TileOffset.CENTER_RIGHT else TileOffset.CENTER_LEFT).withHeading(heading, AngleUnit.DEGREES)
}
