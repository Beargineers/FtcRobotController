package org.beargineers.robot

import com.bylazar.configurables.annotations.Configurable

@Configurable
object CameraPosition {
    var forward = 20.0
    var up = 36.0
    var right = 0.0
    var pitch = -90.0
    var yaw = 0.0
    var roll = 0.0
}

@Configurable
object KalmanFilterConfig {
    // Process noise: uncertainty added per cycle (how much we distrust odometry)
    var PROCESS_NOISE_POSITION = 0.5  // cm - odometry position drift per cycle
    var PROCESS_NOISE_HEADING = 0.01  // radians - odometry heading drift per cycle

    // Measurement noise: base uncertainty in vision measurements
    var MEASUREMENT_NOISE_POSITION = 5.0  // cm - vision position measurement noise
    var MEASUREMENT_NOISE_HEADING = 0.1   // radians - vision heading measurement noise
}
