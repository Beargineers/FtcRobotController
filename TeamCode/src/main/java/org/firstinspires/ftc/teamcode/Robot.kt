package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.internal.RobotOpModeBase

abstract class Robot() : RobotOpModeBase() {
    lateinit var drive: Drivebase
    lateinit var aprilTags: AprilTagWebcam

    override fun init() {
        drive = Drivebase(this)
        aprilTags = AprilTagWebcam(this)

        telemetry.addLine("Ready")
    }

    override fun stop() {
        drive.stop()
        aprilTags.stop()
    }
}