package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.internal.Hardware

class Intake(op: OpMode): Hardware(op) {
    val intake: DcMotor by hardware("intake")

    init {
        intake.direction = DcMotorSimple.Direction.REVERSE
    }

    fun enable(on: Boolean) {
        if (on) {
            setMotorPower(intake, 1.0)
        }
        else {
            setMotorPower(intake, 0.0)
        }
    }

    override fun stop() {
        enable(false)
    }
}