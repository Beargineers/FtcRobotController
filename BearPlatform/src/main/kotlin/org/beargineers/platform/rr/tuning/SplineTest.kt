package org.beargineers.platform.rr.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.beargineers.platform.rr.MecanumDrive
import org.beargineers.platform.rr.PinpointLocalizer
import org.beargineers.platform.rr.tuning.TuningOpModes.SimplePinpointLocalizer

class SplineTest : LinearOpMode() {

    override fun runOpMode() {
        val pl = PinpointLocalizer(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        val drive = MecanumDrive(hardwareMap,  SimplePinpointLocalizer(pl))

        waitForStart()

        val beginPose = Pose2d(0.0, 0.0, 0.0)
        runBlocking(
            drive.actionBuilder(beginPose)
                .splineTo(Vector2d(30.0, 30.0), Math.PI / 2)
                .splineTo(Vector2d(0.0, 60.0), Math.PI)
                .build()
        )
    }
}
