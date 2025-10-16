package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp
class Teleop : Robot() {
    override fun loop() {
        mtr.power = gamepad1.left_stick_y.toDouble()

        telemetry.addData("Elapsed", "$elapsed")
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y)
    }
}