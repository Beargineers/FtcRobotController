package org.beargineers.platform

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.reflect.KProperty

abstract class Hardware(val robot: BaseRobot) {
    val hardwareMap: HardwareMap get() = robot.opMode.hardwareMap
    val telemetry: Telemetry get() = robot.opMode.telemetry

    init {
        robot.registerHardware(this)
    }

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
    inline fun <reified T> getHardware(name: String) = hardwareMap.get(T::class.java, name)

    open fun init() {}
    open fun loop() {}
    open fun stop() {}
}