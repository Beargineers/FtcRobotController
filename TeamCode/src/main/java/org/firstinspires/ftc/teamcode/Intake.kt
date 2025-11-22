package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.internal.Hardware

enum class IntakeMode {
    OFF, ON, REVERSE
}
class Intake(op: OpMode): Hardware(op) {
    val intake: DcMotor by hardware("intake")

    init {
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