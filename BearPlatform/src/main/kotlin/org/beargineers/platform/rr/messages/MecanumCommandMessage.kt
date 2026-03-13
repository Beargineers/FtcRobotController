package org.beargineers.platform.rr.messages

class MecanumCommandMessage(
    var voltage: Double,
    var leftFrontPower: Double,
    var leftBackPower: Double,
    var rightBackPower: Double,
    var rightFrontPower: Double
) {
    var timestamp: Long

    init {
        this.timestamp = System.nanoTime()
    }
}
