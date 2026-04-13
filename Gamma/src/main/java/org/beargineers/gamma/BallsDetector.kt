package org.beargineers.gamma

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.delay
import org.beargineers.platform.Hardware
import org.beargineers.platform.cm
import org.beargineers.platform.config
import org.beargineers.platform.nextTick
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.time.Duration

val LOWER_SENSOR_THRESHOLD by config(7.cm)

private class RollingBoolean(val size: Int) {
    private val data = BooleanArray(size)
    private var curIdx = 0

    fun update(b: Boolean) {
        data[curIdx++] = b
        if (curIdx == size) curIdx = 0
    }

    fun count(): Int {
        return data.count { it }
    }
}

private class SensorReader(val reader: () -> Boolean, val bot: GammaRobot) {
    private var attentionFramesRemained = 5
    private var framesToSkipRemained = 0
    private var previousState = false
    private val updater = RollingBoolean(20)
    fun state(): Boolean {
        if (--framesToSkipRemained > 0) return previousState

        val newState = reader()
        attentionFramesRemained = if (newState != previousState || bot.isShooting()) 5 else 1
        if (--attentionFramesRemained <= 0) {
            framesToSkipRemained = 5
        }

        previousState = newState
        return newState
    }

    fun hasArtifact(): Boolean {
        updater.update(previousState)
        return updater.count() > 13
    }
}

class BallsDetector(val bot: GammaRobot) : Hardware(bot) {
    private val upperSensor by hardware<DigitalChannel>()
    private val lowerSensor by hardware<RevColorSensorV3>()

    private val upperReader = SensorReader({ upperSensor.state }, bot)
    private val lowerReader = SensorReader({ lowerSensor.getDistance(DistanceUnit.CM).cm < LOWER_SENSOR_THRESHOLD}, bot)

    private val lastSeenBall = ElapsedTime()
    private var artifactsCount = 0

    override fun init() {
        upperSensor.mode = DigitalChannel.Mode.INPUT
    }

    override fun loop() {
        val oldArtifactsCount = artifactsCount

        val seenLower = lowerReader.state()
        val seenUpper = upperReader.state()

        if (upperReader.hasArtifact()) {
            artifactsCount = 2
            if (lowerReader.hasArtifact()) {
                artifactsCount = 3

                if (oldArtifactsCount < 3) {
                    bot.intake.cutoff()
                }
            }
        }

        if (seenUpper || seenLower) {
            if (bot.shooter.pusherActive) {
                bot.shooter.abortShooting()
            }

            if (artifactsCount == 0) {
                artifactsCount = 1
            }

            lastSeenBall.reset()
        }
    }

    fun reset() {
        artifactsCount = 0
    }

    fun artifactsCount(): Int {
        return artifactsCount
    }

    suspend fun waitTillNoBalls(delay: Duration) {
        do {
            delay(delay)
            bot.nextTick()
        } while(bot.isShooting() && lastSeenBall.milliseconds() < delay.inWholeMilliseconds)
    }
}