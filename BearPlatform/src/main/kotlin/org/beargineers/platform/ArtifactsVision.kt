package org.beargineers.platform

import android.graphics.Color
import android.util.Size
import com.bylazar.camerastream.PanelsCameraStream
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorRange

private const val CAMERA_W = 320
private const val CAMERA_H = 240

val INTAKE_WIDTH by config(15.inch)

class ArtifactsVision(robot: BaseRobot, val upsideDown: Boolean) : Hardware(robot) {
    val purpleLocator: ColorBlobLocatorProcessor
    val greenLocator: ColorBlobLocatorProcessor
    val visionPortal: VisionPortal
    private val cameraResolution = Size(CAMERA_W, CAMERA_H)

    init {
        purpleLocator = colorBlobLocator(ColorRange.ARTIFACT_PURPLE)
        greenLocator = colorBlobLocator(ColorRange.ARTIFACT_GREEN)

        visionPortal = VisionPortal.Builder()
            .addProcessor(purpleLocator)
            .addProcessor(greenLocator)
            .setCameraResolution(cameraResolution)
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .build()
    }

    override fun init() {
        PanelsCameraStream.startStream( if (upsideDown) RotatedCameraStreamSource(visionPortal) else visionPortal)
    }

    override fun stop() {
        PanelsCameraStream.stopStream()
        visionPortal.close()
    }

    enum class ArtifactColor {PURPLE, GREEN}
    data class Artifact(val color: ArtifactColor, val px: Float, val py: Float, val radius: Float) {
        val distance: Distance get() = (2073/radius).toDouble().cm

        fun offsetFromCenter(): Distance {
            return (2.5.inch / radius.toDouble()) * (px - CAMERA_W/2).toDouble() // 5 inch is a diameter of an artifact
        }
    }

    fun optimalStrafeDistance(): Distance {
        val raw = artifacts()
        if (raw.isEmpty()) return 0.cm

        // Don't look at artifacts, which appear much farther than first one
        val artifacts = raw.filter { abs(it.distance - raw.first().distance) < 20.cm }.sortedBy { it.px }

        // Now try to take 3 artifacts with minimal movement, then 2 if we can't take 3 at once, then 1

        fun List<Artifact>.groupWidth(): Distance {
            return last().offsetFromCenter() - first().offsetFromCenter()
        }

        fun List<Artifact>.groupOffset(): Distance {
            return (last().offsetFromCenter() + first().offsetFromCenter()) / 2
        }

        fun groupBy(n: Int) = buildList {
            for (i in 0..artifacts.size - n) {
                val group = artifacts.drop(i).take(n)
                if (group.groupWidth() < INTAKE_WIDTH) {
                    add(group)
                }
            }
        }

        val by3 = groupBy(3)
        if (by3.isNotEmpty()) {
            return by3.minBy { abs(it.groupOffset()) }.groupOffset()
        }

        val by2 = groupBy(2)
        if (by2.isNotEmpty()) {
            return by2.minBy { abs(it.groupOffset()) }.groupOffset()
        }

        return artifacts.minBy { it.offsetFromCenter() }.offsetFromCenter()
    }


    fun artifacts(): List<Artifact> {
        fun mapX(x: Float): Float = if (upsideDown) cameraResolution.width - 1f - x else x
        fun mapY(y: Float): Float = if (upsideDown) cameraResolution.height - 1f - y else y

        return purpleLocator.blobs.map {
            Artifact(ArtifactColor.PURPLE, mapX(it.circle.x), mapY(it.circle.y), it.circle.radius)
        } + greenLocator.blobs.map {
            Artifact(ArtifactColor.GREEN, mapX(it.circle.x), mapY(it.circle.y), it.circle.radius)
        }.sortedByDescending { it.radius }
    }

    private fun colorBlobLocator(colorRange: ColorRange): ColorBlobLocatorProcessor = ColorBlobLocatorProcessor.Builder()
        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
//        .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
        .setDrawContours(true) // Show contours on the Stream Preview
        .setBoxFitColor(0) // Disable the drawing of rectangles
        .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
        .setBlurSize(5) // Smooth the transitions between different colors in image
        // the following options have been added to fill in perimeter holes.

        .setDilateSize(15) // Expand blobs to fill any divots on the edges
        .setErodeSize(15) // Shrink blobs back to original size
        .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
        .setTargetColorRange(colorRange)
        .build().apply {
            addFilter(ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1.0))
            addFilter(ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50.0, 20000.0))
        }
}
