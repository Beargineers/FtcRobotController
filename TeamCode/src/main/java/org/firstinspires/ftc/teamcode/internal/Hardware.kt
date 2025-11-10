package org.firstinspires.ftc.teamcode.internal

import com.qualcomm.robotcore.hardware.HardwareMap
import kotlin.reflect.KProperty

abstract class Hardware(val hardwareMap: HardwareMap) {
    class HardwareDelegate<T>(val name: String, val klass: Class<T>) {

        private var value: T? = null
        operator fun getValue(thisRef: Hardware, property: KProperty<*>): T {
            if (value == null) {
                value = thisRef.hardwareMap.get(klass, name.ifEmpty { property.name })
            }
            return value!!
        }
    }

    inline fun <reified T> hardware(name: String = "") = HardwareDelegate(name, T::class.java)
}