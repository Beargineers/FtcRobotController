package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.external.samples.USE_WEBCAM
import org.firstinspires.ftc.teamcode.internal.Hardware
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class AprilTagWebcam(op: OpMode): Hardware(op) {

    fun initAprilTag() {
        // Create the AprilTag processor
        aprilTag = AprilTagProcessor.Builder().build()

        // Adjust Image Decimation to trade-off detection-range for detection-rate
        aprilTag.setDecimation(2.0f)

        // Create the vision portal
        visionPortal = if (USE_WEBCAM) {
            VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
                .addProcessor(aprilTag)
                .build()
        } else {
            VisionPortal.Builder()
                .setCamera(BuiltinCameraDirection.BACK)
                .addProcessor(aprilTag)
                .build()
        }
    }

    private lateinit var visionPortal: VisionPortal
    private lateinit var aprilTag: AprilTagProcessor

    // State for camera initialization
    private enum class CameraState { NOT_STARTED, WAITING, READY }
    private var cameraState = CameraState.NOT_STARTED


    // returns a bolean of wheather it has found the April tag with the ID iputed, and then a list -> [ tag id, name of the tag, range(in cm), bearing (˚), yaw(˚) ], and adds telemetry for those values
    fun GetAprilReadings(tagID: Int): Pair<Boolean, List<Any?>> {
        var targetFound = false
        var desiredTag: AprilTagDetection? = null

        for (detection in aprilTag.detections) {
            // Look to see if we have size info on this tag
            if (detection.metadata != null) {
                // Check to see if we want to track towards this tag
                if ((tagID < 0) || (detection.id == tagID)) {
                    targetFound = true
                    desiredTag = detection
                    break
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id)
                }
            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id)
            }
        }

        if (targetFound) {
            val results = listOf(desiredTag?.id, desiredTag?.metadata?.name, desiredTag?.ftcPose?.range?.times(2.54), desiredTag?.ftcPose?.bearing, desiredTag?.ftcPose?.yaw)
            telemetry.addData("Found", "ID %d (%s)", results[0], results[1])
            telemetry.addData("Range", "%5.1f cm", results[2])
            telemetry.addData("Bearing", "%3.0f degrees", results[3])
            telemetry.addData("Yaw", "%3.0f degrees", results[4])
            return Pair(true, results)
        }else{
            telemetry.addData("Found","No april tag to be seen here ¯\\_(ツ)_/¯")
            return Pair(false, listOf(0,0,0,0,0))
        }

    }

}