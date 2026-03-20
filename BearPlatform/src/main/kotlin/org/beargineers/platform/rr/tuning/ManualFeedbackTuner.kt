package org.beargineers.platform.rr.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.runBlocking
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.beargineers.platform.rr.PinpointLocalizer
import org.beargineers.platform.rr.RRMecanumDrive
import org.beargineers.platform.rr.tuning.TuningOpModes.SimplePinpointLocalizer

@Autonomous(group = "Tune")
class ManualFeedbackTuner : LinearOpMode() {
    val DISTANCE: Double = 64.0

    override fun runOpMode() {
        val pl = PinpointLocalizer(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        val drive = RRMecanumDrive(hardwareMap,  SimplePinpointLocalizer(pl))

        waitForStart()

        while (opModeIsActive()) {
            runBlocking(
                drive.actionBuilder(Pose2d(0.0, 0.0, 0.0))
                    .lineToX(DISTANCE)
                    .lineToX(0.0)
                    .build()
            )
        }
    }
}
