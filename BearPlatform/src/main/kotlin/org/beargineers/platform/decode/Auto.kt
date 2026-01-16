package org.beargineers.platform.decode

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.beargineers.platform.Alliance

@Configurable
// Starting,Shooting positions and angles
object AutonomousPrograms {
    var RedSouth  = "D1:+180,D1:+160"
    var BlueSouth = "C1:+180,C1:-160"
    var RedNorth  = "F6BL:+135,D4:+135"
    var BlueNorth = "A6BR:-135,C4:-135"
}

@Autonomous
class RedSouth : DecodeAutoStrategy(
    Alliance.RED,
    AutonomousPrograms.RedSouth
)

@Autonomous
class RedNorth : DecodeAutoStrategy(
    Alliance.RED,
    AutonomousPrograms.RedNorth
)

@Autonomous
class BlueSouth : DecodeAutoStrategy(
    Alliance.BLUE,
    AutonomousPrograms.BlueSouth
)

@Autonomous
class BlueNorth : DecodeAutoStrategy(
    Alliance.BLUE,
    AutonomousPrograms.BlueNorth
)


