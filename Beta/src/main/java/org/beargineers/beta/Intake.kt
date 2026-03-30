package org.beargineers.beta

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.beargineers.platform.Frame
import org.beargineers.platform.Hardware
import org.beargineers.platform.config
import org.beargineers.platform.counter
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.decode.intakeMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class Intake(val bot: BetaRobot): Hardware(bot) {
    private val intake: DcMotor by hardware("intake")
    private val ballDetector: DistanceSensor by hardware("ball")

    val ballInThreshold by config(13.0)
    val ballOutThreshold by config(17.0)


    val ballCounter = BallCounter(ballInThreshold, ballOutThreshold) {
        artifacts++
        if (artifacts == 3) {
            bot.intakeMode = IntakeMode.OFF
            robot.opMode.gamepad1.rumble(300)
        }
    }

    var artifacts = 0

    override fun init() {
        intake.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun loop() {
        intake.power = when (bot.intakeMode) {
            IntakeMode.ON -> {
                1.0
            }

            IntakeMode.OFF -> {
                0.0
            }

            IntakeMode.REVERSE -> {
                -1.0
            }
        }

        if (!bot.opMode.isFpsLow()) {
            if (bot.intakeMode == IntakeMode.ON && !bot.isShooting() && artifacts < 3) {
                ballCounter.update { ballDetector.getDistance(DistanceUnit.CM) }
            }

            Frame.addData("Artifacts", artifacts)

            bot.ledIndicator.setBasePattern(counter(artifacts, 'G'))
        }
        else {
            Frame.addData("Artifacts", "Low FPS!!!")
        }
    }
    fun onShoot(){
        artifacts = 0
        bot.intakeMode = IntakeMode.ON
        robot.opMode.gamepad1.rumble(300)
    }

    override fun stop() {
        bot.intakeMode = IntakeMode.OFF
    }


class BallCounter(val ballInThreshold: Double, val ballOutThreshold: Double, val onArtifact: () -> Unit) {
    var ballIsIn = false
    var attentionCounter = 0 // Tells for how many more loops we have to pay close attention to sensor data
    var loopsToSkip = 0 // Tells how many more loops it is safe to skip asking sensor for data


    fun update(distanceFn: () -> Double) {
        loopsToSkip--
        if (loopsToSkip >= 0) return

        val distance = distanceFn()

        if (!ballIsIn && distance < ballInThreshold) {
            ballIsIn = true
            attentionCounter = 2
            onArtifact()
        }
        else if (distance > ballOutThreshold) {
            ballIsIn = false
            attentionCounter--
        }

        loopsToSkip = if (attentionCounter > 0) 0 else 2
    }
}}
