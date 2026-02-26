package org.beargineers.platform

abstract class RobotTest {
    lateinit var opMode: TestOpMode
    lateinit var robot: TestRobot

    @org.junit.Before
    fun initOpMode() {
        opMode = TestOpMode()
        robot = TestRobot(opMode)
    }
}