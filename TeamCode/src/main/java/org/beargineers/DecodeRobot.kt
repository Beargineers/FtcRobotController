package org.beargineers

import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Position
import org.beargineers.platform.RobotOpMode
import kotlin.math.abs

class DecodeRobot(opMode: RobotOpMode<DecodeRobot>) : BaseRobot(opMode) {
    override val drive = Drivebase(this)
    val aprilTags = AprilTagWebcam(this)
    val shooter = Shooter(this)
    val intake = Intake(this)

    var aprilTagPose: Position? = null
    var goalDistanceCM: Double? = null
    var savedGoalDistanceCM: Double = 90.0

    override fun loop() {
        super.loop()

        val driveChange = currentMove

        if (abs(driveChange.forward) > 0.1 ||
            abs(driveChange.right) > 0.1 ||
            abs(driveChange.turn) > 0.1) {
            goalDistanceCM = null
        }

        val target = aprilTags.findTarget(opMode.alliance)
        if (target != null) {
            aprilTagPose = target.robotPose()
            goalDistanceCM = target.ftcPose.range
        }

        telemetry.addData("Goal distance", goalDistanceCM ?: "goal not found")

    }
}