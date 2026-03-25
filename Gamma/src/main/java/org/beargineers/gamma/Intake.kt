package org.beargineers.gamma

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.beargineers.platform.Frame
import org.beargineers.platform.Hardware
import org.beargineers.platform.decode.IntakeMode

class Intake(val bot: GammaRobot): Hardware(bot) {
    private val intake: DcMotor by hardware("intake")
    var mode: IntakeMode = IntakeMode.OFF

    override fun init() {
        intake.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun loop() {
        Frame.addData("Intake", mode.name)

        intake.power = when (mode) {
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
        mode=IntakeMode.ON
        robot.opMode.gamepad1.rumble(300)
    }
}