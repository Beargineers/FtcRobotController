package org.beargineers

import org.beargineers.platform.Alliance
import org.beargineers.platform.PhasedAutonomous
import org.beargineers.platform.RobotOpMode

open class DecodeOpMode(alliance: Alliance) : RobotOpMode<DecodeRobot>(alliance) {
    override fun createRobot(opMode: RobotOpMode<DecodeRobot>): DecodeRobot {
        return DecodeRobot(this)
    }
}

abstract class DecodeAutonomous(alliance: Alliance) : PhasedAutonomous<DecodeRobot>(alliance) {
    override fun createRobot(opMode: RobotOpMode<DecodeRobot>): DecodeRobot {
        return DecodeRobot(this)
    }
}