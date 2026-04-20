package org.beargineers.gamma

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlinx.coroutines.delay
import org.beargineers.platform.DoubleMedian
import org.beargineers.platform.Frame
import org.beargineers.platform.Hardware
import org.beargineers.platform.config
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.decode.intakeMode
import org.beargineers.platform.motorPower
import org.beargineers.platform.submitJob
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.time.Duration.Companion.milliseconds

private val ONE_ARTIFACTS_CURRENT_THRESHOLD by config(1500) // Typical current ~1900ma +-60
private val TWO_ARTIFACTS_CURRENT_THRESHOLD by config(2800) // Typical current ~3300ma +-160
private val THREE_ARTIFACTS_CURRENT_THRESHOLD by config(4000) // Typical current ~4500ma +-260

private val INTAKE_CUTOFF_DELAY_MS by config(100)

class Intake(val bot: GammaRobot): Hardware(bot) {
    private val intake by hardware<DcMotor>()
    val motorCurrent = DoubleMedian(10)

    override fun init() {
        intake.direction = DcMotorSimple.Direction.REVERSE
        intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    override fun loop() {
        if (false) {
            motorCurrent.update((intake as DcMotorEx).getCurrent(CurrentUnit.MILLIAMPS))
            Frame.graph("IntakeCurrentMA", motorCurrent.mean())
        }

        if (bot.intakeMode == IntakeMode.REVERSE) {
            bot.ballsDetector.reset()
        }

        intake.motorPower = when (bot.intakeMode) {
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
    }

    fun cutoff() {
        robot.opMode.gamepad1.rumble(300)
        bot.submitJob("Cutoff intake") {
            delay(INTAKE_CUTOFF_DELAY_MS.milliseconds)
            bot.intakeMode = IntakeMode.OFF
        }
    }
}