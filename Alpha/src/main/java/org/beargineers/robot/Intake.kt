package org.beargineers.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Hardware
import org.beargineers.platform.decode.IntakeMode

class Intake(robot: BaseRobot): Hardware(robot) {
    private val intake: DcMotor by hardware("intake")

    var mode: IntakeMode = IntakeMode.OFF

    override fun init() {
        intake.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun loop() {
        telemetry.addData("Intake", mode.name)
        when(mode) {
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
        mode = IntakeMode.OFF
    }
}