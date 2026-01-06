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