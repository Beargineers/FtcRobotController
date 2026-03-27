package org.beargineers.platform

import android.graphics.Color
import android.util.Size
import com.bylazar.camerastream.PanelsCameraStream
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.firstinspires.ftc.vision.opencv.ImageRegion

class ArtifactsVision(robot: BaseRobot, val upsideDown: Boolean) : Hardware(robot) {
    val purpleLocator: ColorBlobLocatorProcessor
    val greenLocator: ColorBlobLocatorProcessor
    val visionPortal: VisionPortal
    private val cameraResolution = Size(320, 240)

    init {
        purpleLocator = colorBlobLocator(ColorRange.ARTIFACT_PURPLE)
        greenLocator = colorBlobLocator(ColorRange.ARTIFACT_GREEN)

        visionPortal = VisionPortal.Builder()
            .addProcessor(purpleLocator)
            .addProcessor(greenLocator)
//            .addProcessor(artifactWatershedProcessor)
            .setCameraResolution(cameraResolution)
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .build()
    }

    override fun init() {
        PanelsCameraStream.startStream( if (upsideDown) RotatedCameraStreamSource(visionPortal) else visionPortal)
    }

    override fun stop() {
        PanelsCameraStream.stopStream()
    }

    override fun loop() {
        val blobs = purpleLocator.blobs + greenLocator.blobs

        val sorted = blobs.sortedByDescending { it.contourArea }

        fun mapX(x: Float): Float = if (upsideDown) cameraResolution.width - 1f - x else x
        fun mapY(y: Float): Float = if (upsideDown) cameraResolution.height - 1f - y else y

        for (blob in sorted) {
            Frame.addData(
                "x,y,r",
                "%.1f,%.1f,%.1f",
                mapX(blob.circle.x),
                mapY(blob.circle.y),
                blob.circle.radius,
            )
        }
    }

    private fun colorBlobLocator(colorRange: ColorRange): ColorBlobLocatorProcessor = ColorBlobLocatorProcessor.Builder()
        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
        .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
        .setDrawContours(true) // Show contours on the Stream Preview
        //.setBoxFitColor(0) // Disable the drawing of rectangles
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
