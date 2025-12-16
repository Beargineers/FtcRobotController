package org.beargineers.platform.decode

import org.beargineers.platform.Robot

interface DecodeRobot : Robot {
    fun intakeMode(mode: IntakeMode)
    fun launch()
    fun enableFlywheel(on: Boolean)

    fun isShooting(): Boolean

    fun park()
}