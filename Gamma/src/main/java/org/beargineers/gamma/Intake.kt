package org.beargineers.gamma

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.beargineers.platform.Hardware
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.decode.intakeMode
import org.beargineers.platform.motorPower

class Intake(val bot: GammaRobot): Hardware(bot) {
    private val intake by hardware<DcMotor>()

    override fun init() {
        intake.direction = DcMotorSimple.Direction.REVERSE
        intake.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    override fun loop() {
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
}