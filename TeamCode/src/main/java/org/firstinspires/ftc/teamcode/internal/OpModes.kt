@file:Suppress("unused")

package org.firstinspires.ftc.teamcode.internal

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.reflect.KProperty

abstract class RobotOpModeBase() : OpMode() {
    class HardwareDelegate<T>(val name: String, val klass: Class<T>) {

        private var value: T? = null
        operator fun getValue(thisRef: RobotOpModeBase, property: KProperty<*>): T {
            if (value == null) {
                value = thisRef.hardwareMap.get(klass, name.ifEmpty { property.name })
            }
            return value!!
        }
    }

    inline fun <reified T> hardware(name: String = "") = HardwareDelegate(name, T::class.java)

    val elapsed = ElapsedTime()

    override fun start() {
        elapsed.reset()
    }
}