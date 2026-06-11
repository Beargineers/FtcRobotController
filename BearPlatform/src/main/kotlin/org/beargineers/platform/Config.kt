package org.beargineers.platform

import com.qualcomm.robotcore.hardware.DcMotorSimple
import java.util.Properties
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KClass

object Config {
    internal var currentConfigText = ""
        private set

    private var configs = Properties()
    private val cache = mutableMapOf<String, Any>()
    private val registry = mutableMapOf<KClass<*>, (String) -> Any>()

    init {
        registerTypes()
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
    private fun <T:Any> value(name: String, default: T?, fn: (String) -> T): T {
        synchronized(cache) {
            return cache.getOrPut(name) {
                val c = configValue(name)
                if (c != null) fn(c) else (default ?: error("$name is not defined in config.properties and default value is not specified"))
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

    fun registerTypes() {
        registerType<Int> { it.toInt() }
        registerType<Double> { it.toDouble() }
        registerType<String> { it }
        registerType<Boolean> {
            when(it.lowercase()) {
                "f", "false", "n", "no" -> false
                "y", "yes", "t", "true" -> true
                else -> error("Unknown value $it for boolean config")
            }
        }

        registerType<DcMotorSimple.Direction> {
            when(it) {
                "forward", "FORWARD", "F" -> DcMotorSimple.Direction.FORWARD
                "reverse", "REVERSE",
                "reversed", "REVERSED", "R" -> DcMotorSimple.Direction.REVERSE
                else -> error("Unknown value $it for motor direction")
            }
        }

        registerType<Position> {
            Position.parse(it)
        }

        registerType<Distance> { it.toDouble().cm }
        registerType<Angle> {it.toDouble().degrees}

        registerType<Location> {
            Location.parse(it)
        }

        registerType<PIDFTCoeffs> {
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

    inline fun <reified T : Any> registerType(noinline fn: (String) -> T) {
        registerType(T::class, fn)
    }

    inline fun <reified T : Enum<T>> registerType() {
        registerType(T::class) { name ->
            T::class.java.enumConstants?.find { it.name == name } ?: error("Can't find enum entry named $name")
        }
    }

    fun <T:Any> registerType(typ: KClass<T>, fn: (String) -> T) {
        registry[typ] = fn
    }

    fun <T : Any> configProperty(default: T?, typ: KClass<T>): ReadOnlyProperty<Any?, T> {
        @Suppress("UNCHECKED_CAST")
        val serial = registry[typ] as? (String) -> T ?: error("No config serializer is registered for type $typ")
        return ReadOnlyProperty { _, property ->
            value(property.name, default, serial)
        }
    }
}

inline fun <reified T : Any> config(default: T? = null): ReadOnlyProperty<Any?, T> {
    return Config.configProperty(default, T::class)
}

fun config(dx: Distance, dy: Distance, dh: Angle): ReadOnlyProperty<Any?, Position> {
    return config(Position(dx, dy, dh))
}

fun config(dx: Distance, dy: Distance): ReadOnlyProperty<Any?, Location> {
    return config(Location(dx, dy))
}