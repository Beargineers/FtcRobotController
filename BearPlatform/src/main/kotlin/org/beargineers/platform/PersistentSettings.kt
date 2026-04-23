package org.beargineers.platform

import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import java.util.Properties

object PersistentSettings {
    private val file = AppUtil.getInstance().getSettingsFile("BearPlatform")
    private val properties = Properties()

    init {
        if (file.exists()) {
            file.inputStream().use { properties.load(it) }
        }
    }

    fun getValue(key: String, default: String): String {
        synchronized(this) {
            return (properties[key] ?: default) as String
        }
    }

    fun setValue(key: String, value: String) {
        synchronized(PersistentSettings) {
            properties[key] = value
            file.outputStream().use {
                properties.store(it, "BearPlatform persistent settings storage")
            }
        }
    }
}