package org.beargineers

import android.util.Size
import com.bylazar.configurables.annotations.Configurable
import org.beargineers.platform.ANGLE_UNIT
import org.beargineers.platform.Alliance
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.DISTANCE_UNIT
import org.beargineers.platform.Hardware
import org.beargineers.platform.Position
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

@Configurable
object CameraPosition {
    var forward = 20.0
    var up = 36.0
    var right = 0.0
    var pitch = -90.0
    var yaw = 0.0
    var roll = 0.0
}

class AprilTagWebcam(op: BaseRobot): Hardware(op) {
    private val camera: WebcamName by hardware("Webcam 1")

    private lateinit var aprilTag: AprilTagProcessor
    private lateinit var visionPortal: VisionPortal

    override fun init() {
        super.init()

        val cameraPosition = org.firstinspires.ftc.robotcore.external.navigation.Position(
            DistanceUnit.CM,
            CameraPosition.right, CameraPosition.forward, CameraPosition.up, 0
        )

        val cameraOrientation = YawPitchRollAngles(
            AngleUnit.DEGREES,
            CameraPosition.yaw, CameraPosition.pitch, CameraPosition.roll, 0
        )


        aprilTag = AprilTagProcessor.Builder()
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setOutputUnits(DISTANCE_UNIT, ANGLE_UNIT)
            .setCameraPose(cameraPosition, cameraOrientation)
            .build()

        visionPortal = VisionPortal.Builder()
            .setCamera(camera)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .setCameraResolution(Size(1920, 1080))
            //.setCameraResolution(Size(640, 480))
            .enableLiveView(true)
            .addProcessor(aprilTag)
            .build()

        // Adjust Image Decimation to trade-off detection-range for detection-rate
        aprilTag.setDecimation(3.0f)
    }

    fun findTarget(alliance: Alliance): AprilTagDetection? {
        return when (alliance) {
            Alliance.RED -> findRedTarget()
            Alliance.BLUE -> findBlueTarget()
        }
    }

    fun findRedTarget() : AprilTagDetection? {
        return getAprilReadings(24)
    }

    fun findBlueTarget() : AprilTagDetection? {
        return getAprilReadings(20)
    }

    /**
     *  Returns a detected tag data if found by tagID or first found tag if tagID == -1 or null if no such tag detected
     */
    fun getAprilReadings(tagID: Int): AprilTagDetection? {
        return aprilTag.detections.find {
            it.metadata != null && (tagID < 0 || it.id == tagID)
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

    override fun stop() {
        visionPortal.close()
    }
}

fun AprilTagDetection.robotPose() : Position {
    val rp = this.robotPose
    val offset = ANGLE_UNIT.fromDegrees(90.0)
    return Position(
        rp.position.x,
        rp.position.y,
        rp.orientation.getYaw(ANGLE_UNIT) + offset,
        rp.position.unit,
        ANGLE_UNIT
    )
}
