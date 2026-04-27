package org.beargineers.platform

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.RobotLog
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.util.concurrent.ConcurrentHashMap

object Frame {
    internal var telemetry: Telemetry? = null
    internal var panelsTelemetry: TelemetryManager? = null
    internal var isTestMode = false


    private val TAG = "BEARS"
    fun error(throwable: Throwable, message: String) {
        RobotLog.ee(TAG, throwable, message)
        telemetry?.let {
            it.addLine("FATAL: $message")
            it.update()
        }
    }

    private val messages = ConcurrentHashMap<String, ElapsedTime>()

    fun log(tag: String, msg: String) {
        log(tag) {msg}
    }

    fun log(tag: String, msg: () -> String) {
        if ((messages[tag]?.seconds() ?: 100.0) > 1.0) {
            messages.getOrPut(tag) { ElapsedTime() }.reset()
            log(msg())
        }
    }

    fun log(msg: String) {
        if (isTestMode) println(msg) else RobotLog.i(msg)
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