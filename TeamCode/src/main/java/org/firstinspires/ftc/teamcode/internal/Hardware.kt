package org.firstinspires.ftc.teamcode.internal

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.reflect.KProperty

abstract class Hardware(val op: OpMode) {
    val hardwareMap: HardwareMap get() = op.hardwareMap
    val telemetry: Telemetry get() = op.telemetry

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

    fun setMotorPower(motor: DcMotor, power: Double, compensate: Boolean = true) {
        val nominalVoltage = 12.0
        val voltage = if (compensate) hardwareMap.voltageSensor.iterator().next().voltage else nominalVoltage
        val compensation = voltage / nominalVoltage
        motor.power = (power / compensation).coerceIn(-1.0, 1.0)
    }

    open fun init() {}
    open fun loop() {}
    open fun stop() {}
}