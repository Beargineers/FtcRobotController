package org.beargineers

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.beargineers.platform.Alliance
import org.beargineers.platform.decode.Spike

@Configurable
// Starting,Shooting positions and angles
object AutonomousPrograms {
    var RedSouth  = "D1:+180,D1:+160"
    var BlueSouth = "C1:+180,C1:-160"
    var RedNorth  = "F6BL:0,D4:+135"
    var BlueNorth = "A6BR:0,C4:-135"
}

@Autonomous
class RedSouth : DecodeAutoStrategy(
    Alliance.RED,
    AutonomousPrograms.RedSouth,
    Spike.RIGHT1, Spike.RIGHT2, Spike.RIGHT3)

@Autonomous
class RedNorth : DecodeAutoStrategy(
    Alliance.RED,
    AutonomousPrograms.RedNorth,
    Spike.RIGHT3, Spike.RIGHT2, Spike.RIGHT1)

@Autonomous
class BlueSouth : DecodeAutoStrategy(
    Alliance.BLUE,
    AutonomousPrograms.BlueSouth,
    Spike.LEFT1, Spike.LEFT2, Spike.LEFT3)

@Autonomous
class BlueNorth : DecodeAutoStrategy(
    Alliance.BLUE,
    AutonomousPrograms.BlueNorth,
    Spike.LEFT3, Spike.LEFT2, Spike.LEFT1)
