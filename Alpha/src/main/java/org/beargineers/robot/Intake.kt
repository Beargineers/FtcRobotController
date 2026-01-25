package org.beargineers.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.beargineers.platform.Hardware
import org.beargineers.platform.decode.IntakeMode

class Intake(val alphaRobot: AlphaRobot): Hardware(alphaRobot) {
    private val intake: DcMotor by hardware("intake")

    var mode: IntakeMode = IntakeMode.OFF

    override fun init() {
        intake.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun loop() {
        telemetry.addData("Intake", mode.name)
        when(mode) {
            IntakeMode.ON -> {
                alphaRobot.setMotorPower(intake, 1.0)
            }
            IntakeMode.OFF -> {
                alphaRobot.setMotorPower(intake, 0.0)
            }
            IntakeMode.REVERSE -> {
                alphaRobot.setMotorPower(intake, -1.0)
            }
        }
    }

    override fun stop() {
        mode = IntakeMode.OFF
    }
}