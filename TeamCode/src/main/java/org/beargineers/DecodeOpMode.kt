package org.beargineers

import org.beargineers.platform.Alliance
import org.beargineers.platform.PhasedAutonomous
import org.beargineers.platform.Phases
import org.beargineers.platform.RobotOpMode

open class DecodeOpMode(alliance: Alliance) : RobotOpMode<DecodeRobot>(alliance) {
    override fun createRobot(opMode: RobotOpMode<DecodeRobot>): DecodeRobot {
        return DecodeRobot(this)
    }
}

abstract class DecodeAutonomous(alliance: Alliance, phases: Phases<DecodeRobot>) : PhasedAutonomous<DecodeRobot>(alliance, phases) {
    override fun createRobot(opMode: RobotOpMode<DecodeRobot>): DecodeRobot {
        return DecodeRobot(this)
    }
}