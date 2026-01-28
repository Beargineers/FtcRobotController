package org.beargineers.platform

import org.firstinspires.ftc.robotcore.external.Func
import org.firstinspires.ftc.robotcore.external.Telemetry

class TestOpMode : RobotOpMode<Robot>(Alliance.RED) {

}

class TestRobot(override val opMode: TestOpMode) : Robot {
    override fun <T : Any> getPart(part: Part<T>): T {
        TODO("Not yet implemented")
    }

    override fun driveToTarget(target: Position): Boolean {
        TODO("Not yet implemented")
    }

    override fun drive(
        forwardPower: Double,
        rightPower: Double,
        turnPower: Double
    ) {
        TODO("Not yet implemented")
    }

    override fun driveByPowerAndAngle(
        theta: Double,
        power: Double,
        turn: Double
    ) {
        TODO("Not yet implemented")
    }

    override fun stopDriving() {
    }

    override fun isMoving(): Boolean {
        TODO("Not yet implemented")
    }

    override fun assumePosition(position: Position) {
        TODO("Not yet implemented")
    }

    override fun stop() {
        TODO("Not yet implemented")
    }

    override fun init() {
        TODO("Not yet implemented")
    }

    override fun loop() {
        TODO("Not yet implemented")
    }

    override val telemetry: Telemetry = TestTelemetry()

    override val currentPosition: Position
        get() = TODO("Not yet implemented")
    override val currentVelocity: RelativePosition
        get() = TODO("Not yet implemented")
    override var targetSpeed: Double
        get() = TODO("Not yet implemented")
        set(value) {}
}

class TestTelemetry : Telemetry {
    override fun addData(
        caption: String?,
        format: String?,
        vararg args: Any?
    ): Telemetry.Item? {
        return null
    }

    override fun addData(
        caption: String?,
        value: Any?
    ): Telemetry.Item? {
        return null
    }

    override fun <T : Any?> addData(
        caption: String?,
        valueProducer: Func<T?>?
    ): Telemetry.Item? {
        return null
    }

    override fun <T : Any?> addData(
        caption: String?,
        format: String?,
        valueProducer: Func<T?>?
    ): Telemetry.Item? {
        return null
    }

    override fun removeItem(item: Telemetry.Item?): Boolean {
        return false
    }

    override fun clear() {
    }

    override fun clearAll() {
    }

    override fun addAction(action: Runnable?): Any? {
        return null
    }

    override fun removeAction(token: Any?): Boolean {
        return false
    }

    override fun speak(text: String?) {
    }

    override fun speak(
        text: String?,
        languageCode: String?,
        countryCode: String?
    ) {
    }

    override fun update(): Boolean {
        return true
    }

    override fun addLine(): Telemetry.Line? {
        return null
    }

    override fun addLine(lineCaption: String): Telemetry.Line? {
        println(lineCaption)
        return null
    }

    override fun removeLine(line: Telemetry.Line?): Boolean {
        return false
    }

    override fun isAutoClear(): Boolean {
        return true
    }

    override fun setAutoClear(autoClear: Boolean) {
    }

    override fun getMsTransmissionInterval(): Int {
        return 0
    }

    override fun setMsTransmissionInterval(msTransmissionInterval: Int) {
    }

    override fun getItemSeparator(): String? {
        return ","
    }

    override fun setItemSeparator(itemSeparator: String?) {
    }

    override fun getCaptionValueSeparator(): String? {
        return ":"
    }

    override fun setCaptionValueSeparator(captionValueSeparator: String?) {
    }

    override fun setDisplayFormat(displayFormat: Telemetry.DisplayFormat?) {
    }

    override fun log(): Telemetry.Log? {
        return null
    }
}