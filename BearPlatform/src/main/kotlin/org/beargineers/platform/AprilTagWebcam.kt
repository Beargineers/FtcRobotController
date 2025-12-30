package org.beargineers.platform

import android.util.Size
import com.bylazar.camerastream.PanelsCameraStream
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class AprilTagWebcam(robot: BaseRobot): Hardware(robot), AbsoluteLocalizer {
                         
    // Distance parameters (in cm)
    val CameraPosition_forward by robot.config(0.0)
    val CameraPosition_up by robot.config(0.0)
    val CameraPosition_right by robot.config(0.0)
    val CameraPosition_pitch by robot.config(0.0)
    val CameraPosition_yaw by robot.config(0.0)
    val CameraPosition_roll by robot.config(0.0)


    private val camera: WebcamName by hardware("Webcam 1")

    private lateinit var aprilTag: AprilTagProcessor
    private lateinit var visionPortal: VisionPortal

    override fun init() {
        super.init()

        val cameraPosition = org.firstinspires.ftc.robotcore.external.navigation.Position(
            DistanceUnit.CM,
            CameraPosition_right, CameraPosition_forward, CameraPosition_up, 0
        )

        val cameraOrientation = YawPitchRollAngles(
            AngleUnit.DEGREES,
            CameraPosition_yaw, CameraPosition_pitch, CameraPosition_roll, 0
        )

        aprilTag = AprilTagProcessor.Builder()
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
            .setCameraPose(cameraPosition, cameraOrientation)
            .build()

        visionPortal = VisionPortal.Builder()
            .setCamera(camera)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .setCameraResolution(Size(800, 600))
            //.setCameraResolution(Size(1920, 1080))
            //.setCameraResolution(Size(640, 480))
            .enableLiveView(true)
            .addProcessor(aprilTag)
            .build()

        // Adjust Image Decimation to trade-off detection-range for detection-rate
        aprilTag.setDecimation(3.0f)

        PanelsCameraStream.startStream(visionPortal)
    }

    /**
     *  Returns a detected tag data if found by tagID or all found tags with metadata if tagID == -1 or empty list if no such tag detected
     */
    fun getAprilReadings(tagID: Int): List<AprilTagDetection> {
        return aprilTag.detections.filter {
            it.metadata != null && it.metadata.fieldPosition.magnitude() > 0.01 && (tagID < 0 || it.id == tagID)
        }
    }

    fun addTelemetry(detection: AprilTagDetection?) {
        if (detection != null) {
            telemetry.addData("Found", "ID %d (%s)", detection.id, detection.metadata.name)
            telemetry.addData("Range", "%5.1f cm", detection.ftcPose.range)
            telemetry.addData("Bearing", "%3.0f degrees", detection.ftcPose.bearing)
            telemetry.addData("Yaw", "%3.0f degrees", detection.ftcPose.yaw)
            telemetry.addData("x,y,z", "%5.1f,%5.1f,%5.1f cm", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z)
        } else {
            telemetry.addData("Found","No april tag to be seen here ¯\\_(ツ)_/¯")
        }
    }

    override fun getRobotPose(): Position? {
        return getAprilReadings(-1)
            .map { it.robotPose.robotPose(90.degrees)}
            .firstOrNull()
    }

    override fun stop() {
        visionPortal.close()
        PanelsCameraStream.stopStream()
    }
}

fun Pose3D.robotPose(correction: Angle = 0.degrees): Position {
    return Position(
        DistanceUnit.CM.fromUnit(position.unit, position.x).cm,
        DistanceUnit.CM.fromUnit(position.unit, position.y).cm,
        orientation.getYaw(AngleUnit.DEGREES).degrees + correction
    )
}