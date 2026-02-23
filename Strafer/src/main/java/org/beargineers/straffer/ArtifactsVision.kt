package org.beargineers.straffer

import android.graphics.Color
import android.util.Size
import com.bylazar.camerastream.PanelsCameraStream
import com.qualcomm.robotcore.util.SortOrder
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Hardware
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.firstinspires.ftc.vision.opencv.ImageRegion

class ArtifactsVision(robot: BaseRobot) : Hardware(robot) {
    val purpleLocator: ColorBlobLocatorProcessor
    val greenLocator: ColorBlobLocatorProcessor
    val visionPortal: VisionPortal

    init {
        purpleLocator = colorBlobLocatorProcessorBuilder()
            .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
            .build()

        greenLocator = colorBlobLocatorProcessorBuilder()
            .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
            .build()

        visionPortal = VisionPortal.Builder()
            .addProcessor(purpleLocator)
            .addProcessor(greenLocator)
            .setCameraResolution(Size(320, 240))
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .build()
    }

    override fun init() {
        PanelsCameraStream.startStream(visionPortal)
    }

    override fun stop() {
        PanelsCameraStream.stopStream()
    }

    override fun loop() {
        val blobs = mutableListOf<ColorBlobLocatorProcessor.Blob>()
        blobs += purpleLocator.blobs
        blobs += greenLocator.blobs

        filterAndSort(blobs)

        for (blob in blobs) {
            telemetry.addData("x,y,r", "%.1f,%.1f,%.1f", blob.circle.x, blob.circle.y, blob.circle.radius)
        }
    }

    private fun filterAndSort(blobs: MutableList<ColorBlobLocatorProcessor.Blob>) {
        ColorBlobLocatorProcessor.Util.filterByCriteria(
            ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
            50.0, 20000.0, blobs
        ) // filter out very small blobs.

        ColorBlobLocatorProcessor.Util.filterByCriteria(
            ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
            0.6, 1.0, blobs
        )

        ColorBlobLocatorProcessor.Util.sortByCriteria(
        ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs)
    }

    private fun colorBlobLocatorProcessorBuilder(): ColorBlobLocatorProcessor.Builder = ColorBlobLocatorProcessor.Builder()
        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
        .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
        .setDrawContours(true) // Show contours on the Stream Preview
        .setBoxFitColor(0) // Disable the drawing of rectangles
        .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
        .setBlurSize(5) // Smooth the transitions between different colors in image
        // the following options have been added to fill in perimeter holes.

        .setDilateSize(15) // Expand blobs to fill any divots on the edges
        .setErodeSize(15) // Shrink blobs back to original size
        .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
}