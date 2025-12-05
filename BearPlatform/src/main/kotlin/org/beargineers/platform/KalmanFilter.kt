package org.beargineers.platform

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.PI

/**
 * A Kalman filter for fusing relative (odometry) and absolute (vision) localization data.
 *
 * The Kalman filter maintains an estimate of the robot's position and heading along with
 * uncertainty estimates (covariance). It uses:
 * - Prediction step: Updates position based on relative localizer (odometry) movement
 * - Correction step: Corrects position using absolute localizer (AprilTag vision) when available
 *
 * The filter automatically balances the two sources based on:
 * - Process noise: How much we trust odometry (increases uncertainty over time)
 * - Measurement noise: How much we trust vision (adjusted by confidence)
 *
 * @property processNoisePosition Standard deviation of position drift per update (in cm)
 * @property processNoiseHeading Standard deviation of heading drift per update (in radians)
 * @property measurementNoisePosition Base measurement noise for vision position (in cm)
 * @property measurementNoiseHeading Base measurement noise for vision heading (in radians)
 */
class KalmanFilter(
    private val processNoisePosition: Double = 0.5,  // cm - odometry drift per cycle
    private val processNoiseHeading: Double = 0.01,  // radians - heading drift per cycle
    private val measurementNoisePosition: Double = 5.0,  // cm - vision measurement noise
    private val measurementNoiseHeading: Double = 0.1    // radians - vision heading noise
) {
    // State estimate: [x, y, heading]
    private var stateX: Double = 0.0
    private var stateY: Double = 0.0
    private var stateHeading: Double = 0.0

    // Covariance matrix (3x3, stored as flattened array)
    // Represents uncertainty in [x, y, heading]
    private val covariance = DoubleArray(9) { if (it % 4 == 0) 100.0 else 0.0 }

    /**
     * Initialize or reset the filter with a known position.
     *
     * @param initialPosition The initial robot position
     * @param initialUncertainty Initial uncertainty in position (cm) and heading (radians)
     */
    fun initialize(initialPosition: Position, initialUncertainty: Double = 100.0) {
        val pos = initialPosition.toDistanceUnit(DistanceUnit.CM).toAngleUnit(AngleUnit.RADIANS)
        stateX = pos.x
        stateY = pos.y
        stateHeading = normalizeAngle(pos.heading)

        // Initialize covariance as diagonal matrix with initial uncertainty
        for (i in 0..8) {
            covariance[i] = if (i % 4 == 0) initialUncertainty else 0.0
        }
    }

    /**
     * Prediction step: Update state estimate based on odometry movement.
     *
     * This increases uncertainty (covariance) to account for odometry drift,
     * scaled by the amount of movement. No movement = no additional uncertainty.
     *
     * @param deltaPosition Movement delta from relative localizer
     */
    fun predict(deltaPosition: Position) {
        val delta = deltaPosition.toDistanceUnit(DistanceUnit.CM).toAngleUnit(AngleUnit.RADIANS)

        // Update state estimate (simple addition)
        stateX += delta.x
        stateY += delta.y
        stateHeading = normalizeAngle(stateHeading + delta.heading)

        // Increase covariance (uncertainty grows proportionally to movement)
        // Process noise Q, scaled by movement magnitude
        val positionMovement = kotlin.math.sqrt(delta.x * delta.x + delta.y * delta.y)
        val headingMovement = kotlin.math.abs(delta.heading)

        // Only add process noise proportional to actual movement
        val qPos = processNoisePosition * processNoisePosition * positionMovement
        val qHead = processNoiseHeading * processNoiseHeading * headingMovement

        covariance[0] += qPos  // P[0,0] - x variance
        covariance[4] += qPos  // P[1,1] - y variance
        covariance[8] += qHead // P[2,2] - heading variance
    }

    /**
     * Correction step: Update state estimate using absolute position measurement.
     *
     * This reduces uncertainty by incorporating vision data. The correction amount
     * depends on the relative trust between odometry and vision (based on covariances).
     *
     * @param measurement Absolute position measurement from vision
     * @return true if correction was applied, false if measurement was rejected
     */
    fun correct(measurement: AbsolutePose): Boolean {
        val meas = measurement.pose.toDistanceUnit(DistanceUnit.CM).toAngleUnit(AngleUnit.RADIANS)

        // Scale measurement noise by confidence (low confidence = high noise)
        // confidence 1.0 -> use base noise, confidence 0.5 -> double the noise
        val confidenceFactor = 1.0 / (measurement.confidence + 0.1) // Avoid division by zero
        val rPos = measurementNoisePosition * confidenceFactor
        val rHead = measurementNoiseHeading * confidenceFactor

        // Measurement noise covariance R (diagonal)
        val rPosVar = rPos * rPos
        val rHeadVar = rHead * rHead

        // Innovation (measurement residual): z - h(x)
        val innovX = meas.x - stateX
        val innovY = meas.y - stateY
        val innovHeading = normalizeAngle(meas.heading - stateHeading)

        // Innovation covariance: S = H * P * H^T + R
        // Since H is identity, S = P + R
        val s00 = covariance[0] + rPosVar  // x
        val s11 = covariance[4] + rPosVar  // y
        val s22 = covariance[8] + rHeadVar // heading

        // Kalman gain: K = P * H^T * S^-1
        // For diagonal S and identity H: K = P / S
        val kx = covariance[0] / s00
        val ky = covariance[4] / s11
        val kh = covariance[8] / s22

        // Update state estimate: x = x + K * innovation
        stateX += kx * innovX
        stateY += ky * innovY
        stateHeading = normalizeAngle(stateHeading + kh * innovHeading)

        // Update covariance: P = (I - K * H) * P
        // For diagonal case: P = (1 - K) * P
        covariance[0] *= (1.0 - kx)
        covariance[4] *= (1.0 - ky)
        covariance[8] *= (1.0 - kh)

        // Ensure covariance doesn't become negative or too small
        covariance[0] = covariance[0].coerceAtLeast(0.01)
        covariance[4] = covariance[4].coerceAtLeast(0.01)
        covariance[8] = covariance[8].coerceAtLeast(0.0001)

        return true
    }

    /**
     * Get the current state estimate.
     *
     * @param distanceUnit Desired distance unit for output
     * @param angleUnit Desired angle unit for output
     * @return Current estimated position
     */
    fun getEstimate(
        distanceUnit: DistanceUnit = DistanceUnit.CM,
        angleUnit: AngleUnit = AngleUnit.RADIANS
    ): Position {
        return Position(
            stateX,
            stateY,
            stateHeading,
            DistanceUnit.CM,
            AngleUnit.RADIANS
        ).toDistanceUnit(distanceUnit).toAngleUnit(angleUnit)
    }

    /**
     * Get the current uncertainty (standard deviation) in position and heading.
     *
     * @return Triple of (x_std, y_std, heading_std) in cm and radians
     */
    fun getUncertainty(): Triple<Double, Double, Double> {
        return Triple(
            kotlin.math.sqrt(covariance[0]),  // x std dev
            kotlin.math.sqrt(covariance[4]),  // y std dev
            kotlin.math.sqrt(covariance[8])   // heading std dev
        )
    }

    /**
     * Normalize angle to [-π, π]
     */
    private fun normalizeAngle(angle: Double): Double {
        var normalized = angle
        while (normalized > PI) normalized -= 2 * PI
        while (normalized < -PI) normalized += 2 * PI
        return normalized
    }
}
