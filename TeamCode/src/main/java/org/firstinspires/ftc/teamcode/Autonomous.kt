package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous

@Autonomous
class Autonomous() : Robot() {
    override fun loop() {
        // As an example this will drive a robot along a circle for 10 seconds then stop
        when {
            elapsed.seconds() < 10 -> {
                drive.drive(0.3, 0.0, 0.2)
            }

            else -> {
                drive.stop()
            }
        }

        telemetry.addData("Elapsed", "$elapsed")
    }
}