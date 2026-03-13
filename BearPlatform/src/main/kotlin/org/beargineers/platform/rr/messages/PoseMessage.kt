package org.beargineers.platform.rr.messages

import com.acmerobotics.roadrunner.Pose2d

class PoseMessage(pose: Pose2d) {
    var timestamp: Long
    var x: Double
    var y: Double
    var heading: Double

    init {
        this.timestamp = System.nanoTime()
        this.x = pose.position.x
        this.y = pose.position.y
        this.heading = pose.heading.toDouble()
    }
}

