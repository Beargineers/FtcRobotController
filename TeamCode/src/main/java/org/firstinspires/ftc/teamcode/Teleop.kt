package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

@TeleOp(name = "Mecanum Drive Kotlin", group = "Drive")
class MecanumTeleOp : Robot() {
    override fun loop() {
        val y = -gamepad1.left_stick_y.normalize()   // forward/back
        val x =  gamepad1.left_stick_x.normalize()   // strafe
        val r =  gamepad1.right_stick_x.normalize()  // rotate

        val slow = gamepad1.right_bumper
        drive.drive(y, x, r, slow)

        telemetry.addData("Mode", if (slow) "SLOW" else "FULL")
        telemetry.addData("y/x/r", "%.2f / %.2f / %.2f", y, x, r)
    }

    fun Float.normalize(): Double = shape(deadband(this.toDouble()))

    private fun deadband(v: Double, th: Double = 0.05) = if (abs(v) < th) 0.0 else v
    private fun shape(v: Double) = sign(v) * abs(v).pow(3.0)
}
