package org.beargineers.platform.rr.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import org.beargineers.platform.Alliance
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Robot
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.rr.MecanumDrive

class SplineTest : RobotOpMode<Robot>() {
    override val alliance: Alliance get() = Alliance.RED
    val drive = MecanumDrive(robot as BaseRobot)
    override fun bearLoop() {
        val beginPose = Pose2d(0.0, 0.0, 0.0)
        runBlocking(
            drive.actionBuilder(beginPose)
                .splineTo(Vector2d(30.0, 30.0), Math.PI / 2)
                .splineTo(Vector2d(0.0, 60.0), Math.PI)
                .build()
        )
    }
}
