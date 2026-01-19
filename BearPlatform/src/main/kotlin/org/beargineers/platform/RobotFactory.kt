package org.beargineers.platform

interface RobotFactory<T:Robot> {
    val configResource: Int

    fun createRobot(op: RobotOpMode<T>): T

    companion object {
        val instance = Class.forName("RobotFab").getDeclaredConstructor().newInstance() as RobotFactory<*>

        @Suppress("UNCHECKED_CAST")
        fun <T: Robot> newRobot(op: RobotOpMode<T>): T {
            return (instance as RobotFactory<T>).createRobot(op)
        }
    }
}
