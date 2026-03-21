package org.beargineers.platform

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.Telemetry

object Frame {
    internal var telemetry: Telemetry? = null
    internal var panelsTelemetry: TelemetryManager? = null


    private val TAG = "BEARS"
    fun error(throwable: Throwable, message: String) {
        RobotLog.ee(TAG, throwable, message)
        telemetry?.let {
            it.addLine("FATAL: $message")
            it.update()
        }
    }

    fun graph(name: String, value: Double) {
        panelsTelemetry?.addData(name, value)
    }

    fun addData(name: String, value: Any) {
        telemetry?.addData(name, value)
        panelsTelemetry?.addData(name, value)
    }

    fun addData(name: String, format: String, vararg args: Any) {
        addData(name, String.format(format, *args) as Any)
    }

    fun addLine(string: String) {
        telemetry?.addLine(string)
        panelsTelemetry?.addLine(string)
    }
}