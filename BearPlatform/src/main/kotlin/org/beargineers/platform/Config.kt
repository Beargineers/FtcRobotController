package org.beargineers.platform

import com.qualcomm.robotcore.hardware.DcMotorSimple
import java.util.Properties
import kotlin.properties.ReadOnlyProperty

object Config {
    internal var currentConfigText = ""
        private set

    private var configs = Properties()

    fun updateConfigText(text: String) {
        currentConfigText = text
        configs = Properties().apply {
            load(text.reader())
        }
    }

    fun configValue(name: String): String? {
        return configs[name] as? String
    }

    fun defaultConfigText(text: String) {
        if (currentConfigText.isBlank()) {
            updateConfigText(text)
        }
    }
}
    

fun config(default: Double) : ReadOnlyProperty<Any, Double> {
    return ReadOnlyProperty {_, property ->
        Config.configValue(property.name)?.toDouble() ?: default
    }
}

fun config(default: Int) : ReadOnlyProperty<Any, Int> {
    return ReadOnlyProperty {_, property ->
        Config.configValue(property.name)?.toInt() ?: default
    }
}

fun config(default: String) : ReadOnlyProperty<Any, String> {
    return ReadOnlyProperty {_, property ->
        Config.configValue(property.name) ?: default
    }
}

fun config(default: DcMotorSimple.Direction) : ReadOnlyProperty<Any, DcMotorSimple.Direction> {
    return ReadOnlyProperty {_, property ->
        Config.configValue(property.name)?.let {
            when(it) {
                "forward", "FORWARD", "F" -> DcMotorSimple.Direction.FORWARD
                "reverse", "REVERSE",
                "reversed", "REVERSED", "R" -> DcMotorSimple.Direction.REVERSE
                else -> null
            }
        } ?: default
    }
}

fun config(dx: Distance, dy: Distance, dh: Angle): ReadOnlyProperty<Any, Position> {
    return config(Position(dx, dy, dh))
}

fun config(default: Position): ReadOnlyProperty<Any, Position> {
    return ReadOnlyProperty { _, property ->
        Config.configValue(property.name)?.let {
            val first = it.first()
            if (first.isDigit() || first=='-') {
                val (x, y, heading) = it.split(",").map { it.trim().toDouble() }
                Position(x.cm, y.cm, heading.degrees)
            }
            else {
                tilePosition(it)
            }
        } ?: default
    }
}

fun config(dx: Distance, dy: Distance): ReadOnlyProperty<Any, Location> {
    return config(Location(dx, dy))
}

fun config(default: Location): ReadOnlyProperty<Any, Location> {
    return ReadOnlyProperty { _, property ->
        Config.configValue(property.name)?.let {
            val first = it.first()
            if (first.isDigit() || first=='-') {
                val (x, y) = it.split(",").map { it.trim().toDouble() }
                Location(x.cm, y.cm)
            }
            else {
                tileLocation(it)
            }
        } ?: default
    }
}