package org.beargineers.platform

@Suppress("unused")
class PanelsConfig : com.bylazar.panels.PanelsConfig() {
    @Transient
    override var isDisabled = false

    @Transient
    override var enableLogs = false

    @Transient
    override var enableClassCallerLogs = false
}

object DevMode {
    private var devMode: Boolean? = null
    fun isDevMode(): Boolean {
        if (devMode == null) {
            devMode = PersistentSettings.getValue("devMode", "false") == "true"
        }
        return devMode!!
    }

    fun setDevMode(value: Boolean) {
        devMode = value
        PersistentSettings.setValue("devMode", if (value) "true" else "false")
    }
}