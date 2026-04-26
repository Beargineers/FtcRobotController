package org.beargineers.platform.rr.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.beargineers.platform.rr.PinpointLocalizer
import org.beargineers.platform.rr.RRMecanumDrive
import org.beargineers.platform.rr.tuning.TuningOpModes.SimplePinpointLocalizer

class LocalizationTest : LinearOpMode() {
    override fun runOpMode() {
        val pl = PinpointLocalizer(hardwareMap, Pose2d(0.0, 0.0, 0.0))
        val drive = RRMecanumDrive(hardwareMap,  SimplePinpointLocalizer(pl))

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry())

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

            drive.updatePoseEstimate()

            val pose = drive.localizer.currentPosition
            telemetry.addData("Position", pose)
            telemetry.update()

/*
            val packet = TelemetryPacket()
            packet.fieldOverlay().setStroke("#3F51B5")
            drawRobot(packet.fieldOverlay(), pose.)
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
*/
        }
    }
}
