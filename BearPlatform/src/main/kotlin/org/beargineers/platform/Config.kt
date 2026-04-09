package org.beargineers.platform

import com.qualcomm.robotcore.hardware.DcMotorSimple
import java.util.Properties
import kotlin.properties.ReadOnlyProperty

object Config {
    internal var currentConfigText = ""
        private set

    private var configs = Properties()
    private val cache = mutableMapOf<String, Any>()

    init {
        SettingsWebServer.start()
    }

    fun updateConfigText(text: String) {
        synchronized(cache) {
            cache.clear()
        }

        currentConfigText = text
        configs = Properties().apply {
            load(text.reader())
        }
    }

    @Suppress("UNCHECKED_CAST")
    fun <T:Any> value(name: String, default: T, fn: (String) -> T): T {
        synchronized(cache) {
            return cache.getOrPut(name) {
                val c = configValue(name)
                if (c != null) fn(c) else default
            } as T
        }
    }

    private fun configValue(name: String): String? {
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
        Config.value(property.name, default) {it.toDouble()}
    }
}

fun config(default: Int) : ReadOnlyProperty<Any?, Int> {
    return ReadOnlyProperty {_, property ->
        Config.value(property.name, default) {it.toInt()}
    }
}

fun config(default: String) : ReadOnlyProperty<Any?, String> {
    return ReadOnlyProperty {_, property ->
        Config.value(property.name, default) {it}
    }
}

fun config(default: DcMotorSimple.Direction) : ReadOnlyProperty<Any?, DcMotorSimple.Direction> {
    return ReadOnlyProperty {_, property ->
        Config.value(property.name, default) {
            when(it) {
                "forward", "FORWARD", "F" -> DcMotorSimple.Direction.FORWARD
                "reverse", "REVERSE",
                "reversed", "REVERSED", "R" -> DcMotorSimple.Direction.REVERSE
                else -> error("Unknown value $it for motor direction")
            }
        }
    }
}

fun config(default: Boolean) : ReadOnlyProperty<Any?, Boolean> {
    return ReadOnlyProperty {_, property ->
        Config.value(property.name, default) {
            when(it.lowercase()) {
                "f", "false", "n", "no" -> false
                "y", "yes", "t", "true" -> true
                else -> error("Unknown value $it for boolean config")
            }
        }
    }
}

fun config(dx: Distance, dy: Distance, dh: Angle): ReadOnlyProperty<Any?, Position> {
    return config(Position(dx, dy, dh))
}

fun config(default: Position): ReadOnlyProperty<Any?, Position> {
    return ReadOnlyProperty { _, property ->
        Config.value(property.name, default) {
            val first = it.first()
            if (first.isDigit() || first=='-') {
                val (x, y, heading) = it.split(",").map { it.trim().toDouble() }
                Position(x.cm, y.cm, heading.degrees)
            }
            else {
                tilePosition(it)
            }
        }
    }
}

fun config(default: Distance): ReadOnlyProperty<Any?, Distance> {
    return ReadOnlyProperty { _, property ->
        Config.value(property.name, default) {it.toDouble().cm}
    }
}

fun config(default: Angle): ReadOnlyProperty<Any?, Angle> {
    return ReadOnlyProperty { _, property ->
        Config.value(property.name, default) {it.toDouble().degrees}
    }
}

fun config(dx: Distance, dy: Distance): ReadOnlyProperty<Any?, Location> {
    return config(Location(dx, dy))
}

fun config(default: Location): ReadOnlyProperty<Any?, Location> {
    return ReadOnlyProperty { _, property ->
        Config.value(property.name, default) {
            val first = it.first()
            if (first.isDigit() || first=='-') {
                val (x, y) = it.split(",").map { it.trim().toDouble() }
                Location(x.cm, y.cm)
            }
            else {
                tileLocation(it)
            }
        }
    }
}

fun config(default: PIDFTCoeffs): ReadOnlyProperty<Any?, PIDFTCoeffs> {
    return ReadOnlyProperty { _, property ->
        Config.value(property.name, default) {
            val components = mutableListOf<Double>()
            components.addAll(it.split(',').map { it.toDouble() })
            repeat(5) { components.add(0.0)}

            PIDFTCoeffs(
                components[0],
                components[1],
                components[2],
                components[3]
            )
        }
    }
}