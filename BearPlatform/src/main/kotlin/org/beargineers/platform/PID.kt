package org.beargineers.platform

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.util.concurrent.TimeUnit
import kotlin.math.abs

data class PIDFTCoeffs(
    val p: Double, val i: Double, val d: Double, val t: Double
)

class PID(
    // Anti-windup parameters
    private val integralMax: Double = Double.POSITIVE_INFINITY,
    private val integralZone: Double = Double.POSITIVE_INFINITY,
    private val outputMin: Double = Double.NEGATIVE_INFINITY,
    private val outputMax: Double = Double.POSITIVE_INFINITY
) {
    private var K: PIDFTCoeffs = PIDFTCoeffs(0.0, 0.0, 0.0, 0.0)

    private var target = 0.0
    private var error = 0.0
    private var integralError = 0.0
    private var derivativeError = 0.0
    private var prevDerivative = 0.0

    private var errorLow = 0.0
    private var errorHigh = 0.0
    private var oscillationPeriod = 0L
    private var updateTimer: ElapsedTime? = null

    // Oscillation detection using derivative sign changes
    private val peakTimes = mutableListOf<Long>()
    private val troughTimes = mutableListOf<Long>()
    private val peakValues = mutableListOf<Double>()
    private val troughValues = mutableListOf<Double>()

    fun setTarget(value: Double) {
        if (value != target) {
            target = value
            reset()
        }
    }

    fun updateCoefficients(k: PIDFTCoeffs) {
        if (this.K != k) {
            this.K = k
            reset()
        }
    }

    private fun reset() {
        error = 0.0
        integralError = 0.0
        derivativeError = 0.0
        prevDerivative = 0.0
        errorLow = 0.0
        errorHigh = 0.0
        oscillationPeriod = 0L
        updateTimer = null
        peakTimes.clear()
        troughTimes.clear()
        peakValues.clear()
        troughValues.clear()
    }

    fun updateCurrent(value: Double) {
        updateError(target - value)
    }

    fun updateError(e: Double) {
        if (updateTimer != null) {
            val dt = updateTimer!!.seconds()
            updateTimer!!.reset()

            prevDerivative = derivativeError
            // Anti-windup Strategy 1: Integral zone (only accumulate when close to target)
            if (abs(e) < integralZone) {
                integralError += e * dt

                // Anti-windup Strategy 2: Clamp integral accumulator
                integralError = integralError.coerceIn(-integralMax, integralMax)
            } else {
                // Reset integral when far from target
                integralError = 0.0
            }

            val de = (e - error) / dt
            derivativeError = K.t * prevDerivative + (1 - K.t) * de

            // Detect oscillation using derivative sign changes
            detectOscillation()
        }
        else {
            updateTimer = ElapsedTime()
        }

        error = e
    }

    private fun detectOscillation() {
        val now: Long = System.nanoTime()
        // Detect when derivative changes sign (peak or trough)
        val derivativeSignChanged = (prevDerivative > 0 && derivativeError < 0) ||
                                    (prevDerivative < 0 && derivativeError > 0)

        if (derivativeSignChanged) {
            if (prevDerivative > 0) {
                // Found a peak (derivative changed from positive to negative)
                peakTimes.add(now)
                peakValues.add(error)

                // Keep only last 3 peaks
                if (peakTimes.size > 3) {
                    peakTimes.removeAt(0)
                    peakValues.removeAt(0)
                }
            } else {
                // Found a trough (derivative changed from negative to positive)
                troughTimes.add(now)
                troughValues.add(error)

                // Keep only last 3 troughs
                if (troughTimes.size > 3) {
                    troughTimes.removeAt(0)
                    troughValues.removeAt(0)
                }
            }

            // Calculate average period from peak-to-peak intervals
            if (peakTimes.size >= 2) {
                val periods = peakTimes.zipWithNext { a, b ->
                    TimeUnit.NANOSECONDS.toMillis(b - a)
                }
                oscillationPeriod = periods.average().toLong()
            }

            // Calculate average magnitude (average peak - average trough)
            if (peakValues.isNotEmpty() && troughValues.isNotEmpty()) {
                errorHigh = peakValues.average()
                errorLow = troughValues.average()
            }
        }
    }

    fun error(): Double = error

    fun result(): Double {
        val output = K.p * error + K.i * integralError + K.d * derivativeError

        // Anti-windup Strategy 3: Back-calculation (conditional integration)
        // If output is saturated and error has same sign as integral, stop accumulating
        val saturated = output < outputMin || output > outputMax
        if (saturated) {
            // Reset integral when output saturates in the same direction as error
            integralError = 0.0
        }

        // Clamp output
        return output.coerceIn(outputMin, outputMax)
    }

    fun logErrors(telemetry: TelemetryManager) {
        telemetry.addData("PE", error * K.p)
        telemetry.addData("DE", derivativeError * K.d)
        telemetry.addData("IE", integralError * K.i)
        telemetry.addData("E", result())
    }

    fun logOscillation(telemetry: TelemetryManager) {
        telemetry.addData("OM", errorHigh - errorLow)
        telemetry.addData("OP", oscillationPeriod)
    }

    fun logPIDState(telemetry: Telemetry) {
        telemetry.addData("Error", "%.3f", error)
        telemetry.addData("P term", "%.3f", K.p * error)
        telemetry.addData("I term", "%.3f", K.i * integralError)
        telemetry.addData("D term", "%.3f", K.d * derivativeError)
        telemetry.addData("Integral accum", "%.1f", integralError)
        telemetry.addData("Output", "%.3f", result())
    }
}