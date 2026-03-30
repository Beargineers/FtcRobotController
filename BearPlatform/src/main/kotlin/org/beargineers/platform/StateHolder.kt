package org.beargineers.platform

import kotlin.reflect.KProperty

class StateHolder<T:Any>(val name: String, val initial: T) {
    operator fun getValue(thisRef: Robot, property: KProperty<*>): T {
        @Suppress("UNCHECKED_CAST")
        return (thisRef as BaseRobot).states.getOrPut(this) { initial } as T
    }

    operator fun setValue(thisRef: Robot, property: KProperty<*>, value: T) {
        (thisRef as BaseRobot).states[this] = value
    }
}