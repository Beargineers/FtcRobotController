package org.beargineers.platform

import android.graphics.Bitmap
import android.graphics.Matrix
import org.firstinspires.ftc.robotcore.external.function.Consumer
import org.firstinspires.ftc.robotcore.external.function.Continuation
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource

class RotatedCameraStreamSource(
    private val delegate: CameraStreamSource,
) : CameraStreamSource {
    override fun getFrameBitmap(continuation: Continuation<out Consumer<Bitmap>>) {
        val rotatedConsumer = Consumer<Bitmap> { bitmap ->
            continuation.dispatch { consumer ->
                consumer.accept(bitmap.rotate180())
            }
        }

        delegate.getFrameBitmap(continuation.createForNewTarget(rotatedConsumer))
    }

    private fun Bitmap.rotate180(): Bitmap {
        val matrix = Matrix().apply {
            postRotate(180f, width / 2f, height / 2f)
        }

        return Bitmap.createBitmap(this, 0, 0, width, height, matrix, true)
    }
}
