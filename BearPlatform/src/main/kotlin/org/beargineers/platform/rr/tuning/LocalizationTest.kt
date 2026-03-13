package org.beargineers.platform.rr.tuning

import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.beargineers.platform.Alliance
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Robot
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.rr.MecanumDrive

@Autonomous(group = "Tune")
class LocalizationTest() : RobotOpMode<Robot>() {
    val drive = MecanumDrive(robot as BaseRobot)
    override val alliance: Alliance get() = Alliance.RED

    override fun bearLoop() {
        drive.setDrivePowers(
            PoseVelocity2d(
                Vector2d(
                    -gamepad1.left_stick_y.toDouble(),
                    -gamepad1.left_stick_x.toDouble()
                ),
                -gamepad1.right_stick_x.toDouble()
            )
        )
    }
}
