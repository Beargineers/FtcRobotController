package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.internal.RobotOpModeBase

abstract class Robot() : RobotOpModeBase() {
    lateinit var drive: Drivebase
    lateinit var aprilTags: AprilTagWebcam
    var pose: Pose2D? = null

    override fun init() {
        drive = Drivebase(this)
        aprilTags = AprilTagWebcam(this)

        telemetry.addLine("Ready")
    }
    override fun loop() {
        pose = aprilTags.robotPose()
        telemetry.addData("Pose", pose)
    }

    override fun stop() {
        drive.stop()
        aprilTags.stop()
    }
}