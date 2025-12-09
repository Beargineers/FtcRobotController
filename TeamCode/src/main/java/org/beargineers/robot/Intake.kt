package org.beargineers.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Hardware

enum class IntakeMode {
    OFF, ON, REVERSE
}
class Intake(robot: BaseRobot): Hardware(robot) {
    val intake: DcMotor by hardware("intake")

    override fun init() {
        intake.direction = DcMotorSimple.Direction.REVERSE
    }

    fun enable(on: IntakeMode) {
        when(on) {
            IntakeMode.ON -> {
                setMotorPower(intake, 1.0)
            }
            IntakeMode.OFF -> {
                setMotorPower(intake, 0.0)
            }
            IntakeMode.REVERSE -> {
                setMotorPower(intake, -1.0)
            }
        }
    }

    override fun stop() {
        enable(IntakeMode.OFF)
    }
}