package org.beargineers.platform.rr.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.beargineers.platform.Alliance
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Robot
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.rr.MecanumDrive

@Autonomous(group = "Tune")
class ManualFeedbackTuner : RobotOpMode<Robot>() {
    val DISTANCE: Double = 64.0
    override val alliance: Alliance get() = Alliance.RED
    val drive = MecanumDrive(robot as BaseRobot)

    override fun bearLoop() {
        runBlocking(
            drive.actionBuilder(Pose2d(0.0, 0.0, 0.0))
                .lineToX(DISTANCE)
                .lineToX(0.0)
                .build()
        )
    }
}
