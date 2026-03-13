package org.beargineers.platform.rr.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.beargineers.platform.rr.MecanumDrive
import org.beargineers.platform.rr.PinpointLocalizer
import org.beargineers.platform.rr.tuning.TuningOpModes.SimplePinpointLocalizer

@Autonomous(group = "Tune")
class LocalizationTest : LinearOpMode() {
    override fun runOpMode() {
        val pl = PinpointLocalizer(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        val drive = MecanumDrive(hardwareMap,  SimplePinpointLocalizer(pl))

        waitForStart()

        while (opModeIsActive()) {
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
}
