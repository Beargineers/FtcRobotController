package org.beargineers.straffer

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.beargineers.TeleopConfigs
import org.beargineers.platform.Alliance
import org.beargineers.platform.RobotOpMode

@TeleOp
class StrafferDriving : RobotOpMode<StrafferRobot>(Alliance.BLUE) {
    override fun createRobot(opMode: RobotOpMode<StrafferRobot>): StrafferRobot {
        return StrafferRobot(opMode)
    }

    override fun bearLoop() {
        super.bearLoop()

        val forward = -gamepad1.left_stick_y.toDouble()
        val strafe = gamepad1.left_stick_x.toDouble()
        val rotation =
            (gamepad1.right_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger) * TeleopConfigs.ROTATION_TRIGGER_REDUCTION).toDouble()

        robot.drive.drive(forward, strafe, rotation, true)
        telemetry.addData("f/s/r", "%.2f / %.2f / %.2f", forward, strafe, rotation)
    }
}