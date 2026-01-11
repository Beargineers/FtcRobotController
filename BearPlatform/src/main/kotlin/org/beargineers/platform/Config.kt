package org.beargineers.platform

import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.properties.ReadOnlyProperty

fun Robot.config(default: Double) : ReadOnlyProperty<Any, Double> {
    return ReadOnlyProperty {_, property ->
        configValue(property.name)?.toDouble() ?: default
    }
}

fun Robot.config(default: Int) : ReadOnlyProperty<Any, Int> {
    return ReadOnlyProperty {_, property ->
        configValue(property.name)?.toInt() ?: default
    }
}

fun Robot.config(default: String) : ReadOnlyProperty<Any, String> {
    return ReadOnlyProperty {_, property ->
        configValue(property.name) ?: default
    }
}

fun Robot.config(default: DcMotorSimple.Direction) : ReadOnlyProperty<Any, DcMotorSimple.Direction> {
    return ReadOnlyProperty {_, property ->
        configValue(property.name)?.let {
            when(it) {
                "forward", "FORWARD", "F" -> DcMotorSimple.Direction.FORWARD
                "reverse", "REVERSE",
                "reversed", "REVERSED", "R" -> DcMotorSimple.Direction.REVERSE
                else -> null
            }
        } ?: default
    }
}

fun Robot.config(dx: Distance, dy: Distance, dh: Angle): ReadOnlyProperty<Any, Position> {
    return config(Position(dx, dy, dh))
}

fun Robot.config(default: Position): ReadOnlyProperty<Any, Position> {
    return ReadOnlyProperty { _, property ->
        configValue(property.name)?.let {
            if (it.first().isDigit()) {
                val (x, y, heading) = it.split(",").map { it.trim().toDouble() }
                Position(x.cm, y.cm, heading.degrees)
            }
            else {
                tilePosition(it)
            }
        } ?: default
    }
}

fun Robot.config(dx: Distance, dy: Distance): ReadOnlyProperty<Any, Location> {
    return config(Location(dx, dy))
}

fun Robot.config(default: Location): ReadOnlyProperty<Any, Location> {
    return ReadOnlyProperty { _, property ->
        configValue(property.name)?.let {
            if (it.first().isDigit()) {
                val (x, y) = it.split(",").map { it.trim().toDouble() }
                Location(x.cm, y.cm)
            }
            else {
                tileLocation(it)
            }
        } ?: default
    }
}