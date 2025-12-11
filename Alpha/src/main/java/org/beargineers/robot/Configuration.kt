package org.beargineers.robot

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.beargineers.platform.WheelsConfig

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

@Configurable
object WheelCorrections {
    var LF: Double = 1.0
    var RF: Double = 0.775
    var LB: Double = 1.0
    var RB: Double = 0.925

    var FORWARD_CM_PER_TICK: Double = 0.0594
    var STRAFE_CM_PER_TICK: Double = 0.069 // 0.07096614583 // 0.049 //

    fun asConfig(): WheelsConfig = WheelsConfig(
        lf_direction = DcMotorSimple.Direction.REVERSE,
        lb_direction = DcMotorSimple.Direction.REVERSE,
        rf_direction = DcMotorSimple.Direction.FORWARD,
        rb_direction = DcMotorSimple.Direction.FORWARD,

        lf_encoder_direction = DcMotorSimple.Direction.REVERSE,
        rf_encoder_direction = DcMotorSimple.Direction.REVERSE,
        lb_encoder_direction = DcMotorSimple.Direction.FORWARD,
        rb_encoder_direction = DcMotorSimple.Direction.FORWARD,

        lf_correction = LF,
        rf_correction = RF,
        lb_correction = LB,
        rb_correction = RB,
        cm_per_tick_forward = FORWARD_CM_PER_TICK,
        cm_per_tick_strafe = STRAFE_CM_PER_TICK
    )
}

@Configurable
object AutonomousConfig {
    var MINIMAL_WHEEL_POWER: Double = 0.12


    var kP_position = 0.035
    var kP_heading = 0.01
}

@Configurable
object ShooterConfig {
    var SHOOTER_POWER_ADJUST = 1.0
    var SHOOTER_DISTANCE_QUOTIENT = 0.00101
    var SHOOTER_FREE_QUOTIENT = 0.556
    var SHOOTING_TIME_SECONDS: Double = 4.5
}
