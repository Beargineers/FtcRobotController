package org.beargineers.platform


interface Drivetrain {
    fun stop()
    fun drive(forwardPower: Double, rightPower: Double, turnPower: Double)
    fun driveByPowerAndAngle(theta: Double, power: Double, turn: Double)
}