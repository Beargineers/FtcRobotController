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

class BallsDetector(val bot: GammaRobot) : Hardware(bot) {
    val upperSensor by hardware<DigitalChannel>()
    val lowerSensor by hardware<RevColorSensorV3>()

    private val lastSeenBall = ElapsedTime()
    private val rollingLower = RollingBoolean(20)
    private val rollingUpper = RollingBoolean(20)
    private var artifactsCount = 0

    override fun init() {
        upperSensor.mode = DigitalChannel.Mode.INPUT
    }

    override fun loop() {
        val oldArtifactsCount = artifactsCount

        val seenLower = lowerSensor.getDistance(DistanceUnit.CM).cm < LOWER_SENSOR_THRESHOLD
        val seenUpper = upperSensor.state

        rollingLower.update(seenLower)
        rollingUpper.update(seenUpper)

        val hasLower = rollingLower.count() > 13
        val hasUpper = rollingUpper.count() > 13

        if (hasUpper) {
            artifactsCount = 2
            if (hasLower) {
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