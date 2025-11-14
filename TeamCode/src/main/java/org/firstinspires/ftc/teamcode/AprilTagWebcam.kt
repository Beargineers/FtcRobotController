package org.firstinspires.ftc.teamcode

import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.teamcode.internal.Hardware
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class AprilTagWebcam(op: OpMode): Hardware(op) {
    private val camera: WebcamName by hardware("Webcam 1")

    private val aprilTag: AprilTagProcessor = AprilTagProcessor.Builder()
        .setDrawTagID(true)
        .setDrawTagOutline(true)
        .setDrawAxes(true)
        .setDrawCubeProjection(true)
        .setOutputUnits(DISTANCE_UNIT, ANGLE_UNIT)
        .build()

    private val visionPortal: VisionPortal = VisionPortal.Builder()
        .setCamera(camera)
        .setCameraResolution(Size(640, 480))
        .enableLiveView(true)
        .addProcessor(aprilTag)
        .build()

    init {
        // Adjust Image Decimation to trade-off detection-range for detection-rate
        aprilTag.setDecimation(2.0f)
    }

    fun findRedTarget() : AprilTagDetection? {
        return getAprilReadings(24)
    }

    fun findBlueTarget() : AprilTagDetection? {
        return getAprilReadings(20)
    }

    /*
    TODO Compensate for camera rotation!
     */
    fun robotPose() : Pose2D? {
        val r = findRedTarget()?.robotPose
        val b = findBlueTarget()?.robotPose

        return when {
            r != null && b != null -> Pose2D(
                (r.position.x + b.position.x) / 2,
                (r.position.y + b.position.y) / 2,
                (r.orientation.yaw + b.orientation.yaw) / 2,
                r.position.unit,
                ANGLE_UNIT
            )

            b != null -> Pose2D(b.position.x, b.position.y, b.orientation.yaw, b.position.unit, ANGLE_UNIT)
            r != null -> Pose2D(r.position.x, r.position.y, r.orientation.yaw, r.position.unit, ANGLE_UNIT)
            else -> null
        }
    }

    /**
     *  Returns a detected tag data if found by tagID or first found tag if tagID == -1 or null if no such tag detected
     */
    fun getAprilReadings(tagID: Int): AprilTagDetection? {
        var desiredTag: AprilTagDetection? = null

        for (detection in aprilTag.detections) {
            // Look to see if we have size info on this tag
            if (detection.metadata != null) {
                // Check to see if we want to track towards this tag
                if ((tagID < 0) || (detection.id == tagID)) {
                    desiredTag = detection
                    break
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id)
                }
            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id)
            }
        }

        if (desiredTag != null) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name)
            telemetry.addData("Range", "%5.1f cm", desiredTag.ftcPose.range)
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing)
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw)
            telemetry.addData("x,y,z", "%5.1f,%5.1f,%5.1f cm", desiredTag.ftcPose.x, desiredTag.ftcPose.y, desiredTag.ftcPose.z)

            return desiredTag
        } else {
            telemetry.addData("Found","No april tag to be seen here ¯\\_(ツ)_/¯")
            return null
        }
    }

    fun stop() {
        visionPortal.close()
    }
}