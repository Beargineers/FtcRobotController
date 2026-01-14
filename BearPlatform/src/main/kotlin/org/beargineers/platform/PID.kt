package org.beargineers.platform

import com.bylazar.telemetry.TelemetryManager
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.util.concurrent.TimeUnit

class PID(
    // Anti-windup parameters
    private val integralMax: Double = Double.POSITIVE_INFINITY,
    private val integralZone: Double = Double.POSITIVE_INFINITY,
    private val outputMin: Double = Double.NEGATIVE_INFINITY,
    private val outputMax: Double = Double.POSITIVE_INFINITY
) {
    private var p: Double = 0.0
    private var i: Double = 0.0
    private var d: Double = 0.0

    private var target = 0.0
    private var error = 0.0
    private var integralError = 0.0
    private var derivativeError = 0.0
    private var prevDerivative = 0.0

    private var errorLow = 0.0
    private var errorHigh = 0.0
    private var oscillationPeriod = 0L
    private var updateTimeNano = 0L

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

    fun updateCoefficients(p: Double, i: Double, d: Double) {
        if (this.p != p || this.i != i || this.d != d) {
            this.p = p
            this.i = i
            this.d = d
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
        updateTimeNano = 0L
        peakTimes.clear()
        troughTimes.clear()
        peakValues.clear()
        troughValues.clear()
    }

    fun updateCurrent(value: Double) {
        updateError(target - value)
    }

    fun updateError(e: Double) {
        val now = System.nanoTime()
        val dtMillis = if (updateTimeNano > 0) TimeUnit.NANOSECONDS.toMillis(now - updateTimeNano) else 0L
        updateTimeNano = now

        if (dtMillis > 0) {
            // Anti-windup Strategy 1: Integral zone (only accumulate when close to target)
            if (kotlin.math.abs(e) < integralZone) {
                integralError += e * dtMillis

                // Anti-windup Strategy 2: Clamp integral accumulator
                integralError = integralError.coerceIn(-integralMax, integralMax)
            } else {
                // Reset integral when far from target
                integralError = 0.0
            }

            derivativeError = (e - error) / dtMillis

            // Detect oscillation using derivative sign changes
            detectOscillation(now, derivativeError)
        }

        error = e
    }

    private fun detectOscillation(now: Long, derivative: Double) {
        // Detect when derivative changes sign (peak or trough)
        val derivativeSignChanged = (prevDerivative > 0 && derivative < 0) ||
                                    (prevDerivative < 0 && derivative > 0)

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

        prevDerivative = derivative
    }

    fun error(): Double = error

    fun result(): Double {
        val output = p * error + i * integralError + d * derivativeError

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
        telemetry.addData("PE", error * p)
        telemetry.addData("DE", derivativeError * d)
        telemetry.addData("IE", integralError * i)
        telemetry.addData("E", result())
    }

    fun logOscillation(telemetry: Telemetry) {
        telemetry.addData("Oscillation magnitude", errorHigh - errorLow)
        telemetry.addData("Oscillation period", oscillationPeriod)
    }

    fun logPIDState(telemetry: Telemetry) {
        telemetry.addData("Error", "%.3f", error)
        telemetry.addData("P term", "%.3f", p * error)
        telemetry.addData("I term", "%.3f", i * integralError)
        telemetry.addData("D term", "%.3f", d * derivativeError)
        telemetry.addData("Integral accum", "%.1f", integralError)
        telemetry.addData("Output", "%.3f", result())
    }
}