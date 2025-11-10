package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.internal.RobotOpModeBase

abstract class Robot() : RobotOpModeBase() {
    lateinit var drive: Drivebase

    override fun init() {
        drive = Drivebase(hardwareMap)

        telemetry.addLine("Ready")
    }

    override fun stop() {
        drive.stop()
    }
}