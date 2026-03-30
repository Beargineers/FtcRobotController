package org.beargineers.gamma

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.beargineers.platform.DoubleNormalDistribution
import org.beargineers.platform.Frame
import org.beargineers.platform.Hardware
import org.beargineers.platform.config
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.decode.intakeMode
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

private val ONE_ARTIFACTS_CURRENT_THRESHOLD by config(1500) // Typical current ~1900ma +-60
private val TWO_ARTIFACTS_CURRENT_THRESHOLD by config(2800) // Typical current ~3300ma +-160
private val THREE_ARTIFACTS_CURRENT_THRESHOLD by config(4000) // Typical current ~4500ma +-260

class Intake(val bot: GammaRobot): Hardware(bot) {
    private val intake: DcMotor by hardware("intake")
    private val motorCurrent = DoubleNormalDistribution(10)

    var artifactsCount = 0

    override fun init() {
        intake.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun loop() {
        motorCurrent.update((intake as DcMotorEx).getCurrent(CurrentUnit.MILLIAMPS))
        val (current, currentStd) = motorCurrent.result()
        Frame.graph("IntakeCurrentMA", current)
        Frame.graph("IntakeCurrentMASTD", currentStd)

        if (bot.intakeMode == IntakeMode.ON) {
            artifactsCount = when {
                current > THREE_ARTIFACTS_CURRENT_THRESHOLD -> 3
                current > TWO_ARTIFACTS_CURRENT_THRESHOLD -> 2
                current > ONE_ARTIFACTS_CURRENT_THRESHOLD ->  1
                else -> 0
            }

/*
            if (artifactsCount == 3) {
                mode = IntakeMode.OFF
                robot.opMode.gamepad1.rumble(300)
            }
*/
        }

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
    }

    fun onShoot() {
        bot.intakeMode = IntakeMode.ON
        robot.opMode.gamepad1.rumble(300)
    }
}