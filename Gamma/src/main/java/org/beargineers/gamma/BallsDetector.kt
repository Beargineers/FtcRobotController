package org.beargineers.gamma

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.delay
import org.beargineers.platform.Frame
import org.beargineers.platform.Hardware
import org.beargineers.platform.cm
import org.beargineers.platform.config
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.decode.IntakeState
import org.beargineers.platform.decode.intakeMode
import org.beargineers.platform.nextTick
import org.beargineers.platform.submitJob
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.time.Duration
import kotlin.time.Duration.Companion.milliseconds

val LOWER_SENSOR_THRESHOLD by config(7.cm)
val INTAKE_CUTOFF_DELAY_MS by config(100)
val OPEN_LATCH_DELAY_MS by config(300)



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

private class SensorReader(val name: String, val reader: () -> Boolean, val bot: GammaRobot) {
    private var attentionFramesRemained = 5
    private var framesToSkipRemained = 0
    private var curState = false
    private val updater = RollingBoolean(15)

    fun state(): Boolean = curState
    fun update() {
        try {
            val isShooting = bot.isShooting()
            if (isShooting) framesToSkipRemained = 0

            if (--framesToSkipRemained > 0) return

            val newState = reader()
            attentionFramesRemained = if (newState != curState || isShooting) 5 else 1
            if (--attentionFramesRemained <= 0) {
                framesToSkipRemained = 5
            }

            curState = newState
        } finally {
            updater.update(curState)
            log()
        }
    }

    fun hasArtifact(): Boolean {
        return updater.count() >= 12
    }

    fun log() {
        Frame.graph("Sensor counts $name", updater.count().toDouble())
    }
}

class BallsDetector(val bot: GammaRobot) : Hardware(bot) {
    private val upperSensor by hardware<DigitalChannel>()
    private val lowerSensor by hardware<RevColorSensorV3>()

    private val upperReader = SensorReader("upper", { upperSensor.state }, bot)
    private val lowerReader = SensorReader("lower", {
        val distance = lowerSensor.getDistance(DistanceUnit.CM)
        Frame.graph("LOWER SENSOR cm", distance)
        val result = distance.cm < LOWER_SENSOR_THRESHOLD
        if (result) {
            Frame.log("LOWER SENSOR SEES: $distance")
        }
        result
                                                    }, bot)

    val lastSeenBall = ElapsedTime()
    private var artifactsCount = 0

    override fun init() {
        upperSensor.mode = DigitalChannel.Mode.INPUT
    }

    fun cutoff() {
        val modCount = IntakeState.modCount
        bot.submitJob("Cutoff intake") {
            delay(INTAKE_CUTOFF_DELAY_MS.milliseconds)
            if (modCount == IntakeState.modCount && artifactsCount == 3) {
                robot.opMode.gamepad1.rumble(300)
                bot.intakeMode = IntakeMode.OFF
            }
        }
    }

    override fun loop() {
        val oldArtifactsCount = artifactsCount

        lowerReader.update()
        upperReader.update()

        if (bot.intakeMode == IntakeMode.REVERSE) {
            artifactsCount = 0
            return
        }

        if (bot.intakeMode == IntakeMode.OFF) {
            if (lowerReader.hasArtifact()) {
                artifactsCount = 3
            }
            else if (upperReader.hasArtifact()) {
                artifactsCount = 2
            }

            return
        }

        val seenLower = lowerReader.state()
        val seenUpper = upperReader.state()

        if (upperReader.hasArtifact()) {
            artifactsCount = 2
            if (lowerReader.hasArtifact()) {
                artifactsCount = 3

                if (oldArtifactsCount < 3) {
                    cutoff()
                }
            }
        }
        else if (artifactsCount > 0) {
            artifactsCount = 1
        }

        if (seenUpper || seenLower) {
            if (bot.shooter.pusherActive) {
                Frame.log("Aborting shooting. seenUpper=$seenUpper, seenLower=$seenLower")
                bot.abortShooting()
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
        Frame.log("Waiting to clear balls")
        do {
            delay(delay)
            bot.nextTick()
        } while(bot.isShooting() && lastSeenBall.milliseconds() < delay.inWholeMilliseconds)
        Frame.log("waitTillNoBalls completed")
    }
}