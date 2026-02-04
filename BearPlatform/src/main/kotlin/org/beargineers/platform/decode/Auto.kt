package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.beargineers.platform.Alliance

@Autonomous
class RedSouth : ProgrammedAuto() {
    override val alliance = Alliance.RED
    override val program = "B00000"
}

@Autonomous
class RedNorth : ProgrammedAuto() {
    override val alliance = Alliance.RED
    override val program = "F2R31"
}

@Autonomous
class BlueSouth : ProgrammedAuto() {
    override val alliance = Alliance.BLUE
    override val program = "B00000"
}

@Autonomous
class BlueNorth : ProgrammedAuto() {
    override val alliance = Alliance.BLUE
    override val program = "F2R31"
}
