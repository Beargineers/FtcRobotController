package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous
class Autonomous() : Robot() {
    override fun loop() {
        mtr.power = (30 - elapsed.seconds()) / 30 // Start at full speed and slowly decrease
        telemetry.addData("Elapsed", "$elapsed")
    }
}