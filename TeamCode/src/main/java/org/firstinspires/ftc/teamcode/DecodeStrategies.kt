package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Configurable
object AutonomousConfig {
    // Max default auto speed. Pass maxSpeed parameter to driveTo to overrun the default
    var MAX_SPEED = 0.7

    // Starting,Shooting positions and angles
    var RedSouth  = "D1:+180,D1:+160"
    var BlueSouth = "C1:+180,C1:-160"
    var RedNorth  = "D4:+90,D4:+141"
    var BlueNorth = "C4:-90,C4:-141"


    // Proportional control gains (tune these values for your robot)
    var kP_position = 0.035  // Position gain (power per distance unit)
    var kP_heading = 0.01   // Heading gain (power per angle unit)
}

@Autonomous
class RedSouth : DecodeAutoStrategy(
    Alliance.RED,
    AutonomousConfig.RedSouth,
    Spike.RIGHT1, Spike.RIGHT2, Spike.RIGHT3)

@Autonomous
class RedNorth : DecodeAutoStrategy(Alliance.BLUE,
    AutonomousConfig.RedNorth,
    Spike.RIGHT3, Spike.RIGHT2, Spike.RIGHT1)

@Autonomous
class BlueSouth : DecodeAutoStrategy(
    Alliance.BLUE,
    AutonomousConfig.BlueSouth,
    Spike.LEFT1, Spike.LEFT2, Spike.LEFT3)

@Autonomous
class BlueNorth : DecodeAutoStrategy(
    Alliance.BLUE,
    AutonomousConfig.BlueNorth,
    Spike.LEFT3, Spike.LEFT2, Spike.LEFT1)
