package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.beargineers.platform.Alliance

@Autonomous
class RedSouth : DecodeAutoStrategy(
    Alliance.RED, ShootingZones.BACK
)

@Autonomous
class RedNorth : DecodeAutoStrategy(
    Alliance.RED, ShootingZones.FRONT
)

@Autonomous
class BlueSouth : DecodeAutoStrategy(
    Alliance.BLUE, ShootingZones.BACK
)

@Autonomous
class BlueNorth : DecodeAutoStrategy(
    Alliance.BLUE, ShootingZones.FRONT
)


