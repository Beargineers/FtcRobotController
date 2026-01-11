package org.beargineers.platform.decode

import org.beargineers.platform.degrees
import org.beargineers.platform.tileLocation

enum class Spike(start: String, end: String) {
    LEFT1("B2", "A2"),
    LEFT2("B3", "A3"),
    LEFT3("B4", "A4"),

    RIGHT1("E2", "F2"),
    RIGHT2("E3", "F3"),
    RIGHT3("E4", "F4");

    val left = name.startsWith("LEFT")
    val heading = (if (left) -90.0 else 90.0).degrees
    val startPose = tileLocation(start + if (left) "CR" else "CL").withHeading(heading)
    val endPose = tileLocation(end + if (left) "CR" else "CL").withHeading(heading)
}