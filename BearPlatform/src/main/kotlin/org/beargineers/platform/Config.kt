package org.beargineers.platform

import com.qualcomm.robotcore.hardware.DcMotorSimple
import java.util.Properties
import kotlin.properties.ReadOnlyProperty

object Config {
    internal var currentConfigText = ""
        private set

    private var configs = Properties()

    init {
        SettingsWebServer.start()
    }

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
    

fun config(default: Double) : ReadOnlyProperty<Any?, Double> {
    return ReadOnlyProperty {_, property ->
        Config.configValue(property.name)?.toDouble() ?: default
    }
}

fun config(default: Int) : ReadOnlyProperty<Any?, Int> {
    return ReadOnlyProperty {_, property ->
        Config.configValue(property.name)?.toInt() ?: default
    }
}

fun config(default: String) : ReadOnlyProperty<Any?, String> {
    return ReadOnlyProperty {_, property ->
        Config.configValue(property.name) ?: default
    }
}

fun config(default: DcMotorSimple.Direction) : ReadOnlyProperty<Any?, DcMotorSimple.Direction> {
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

fun config(default: Boolean) : ReadOnlyProperty<Any?, Boolean> {
    return ReadOnlyProperty {_, property ->
        Config.configValue(property.name)?.let {
            when(it.lowercase()) {
                "f", "false", "n", "no" -> false
                "y", "yes", "t", "true" -> true
                else -> null
            }
        } ?: default
    }
}

fun config(dx: Distance, dy: Distance, dh: Angle): ReadOnlyProperty<Any?, Position> {
    return config(Position(dx, dy, dh))
}

fun config(default: Position): ReadOnlyProperty<Any?, Position> {
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

fun config(default: Distance): ReadOnlyProperty<Any?, Distance> {
    return ReadOnlyProperty { _, property ->
        Config.configValue(property.name)?.toDouble()?.cm ?: default
    }
}

fun config(default: Angle): ReadOnlyProperty<Any?, Angle> {
    return ReadOnlyProperty { _, property ->
        Config.configValue(property.name)?.toDouble()?.degrees ?: default
    }
}

fun config(dx: Distance, dy: Distance): ReadOnlyProperty<Any?, Location> {
    return config(Location(dx, dy))
}

fun config(default: Location): ReadOnlyProperty<Any?, Location> {
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

fun config(default: PIDFTCoeffs): ReadOnlyProperty<Any?, PIDFTCoeffs> {
    return ReadOnlyProperty { _, property ->
        Config.configValue(property.name)?.let {
            val components = mutableListOf<Double>()
            components.addAll(it.split(',').map { it.toDouble() })
            repeat(5) { components.add(0.0)}

            PIDFTCoeffs(
                components[0],
                components[1],
                components[2],
                components[4]
            )

        } ?: default
    }
}