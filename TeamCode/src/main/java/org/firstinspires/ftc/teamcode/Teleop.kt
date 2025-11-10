package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

@TeleOp(name = "Mecanum Drive Kotlin", group = "Drive")
class MecanumTeleOp : LinearOpMode() {
    override fun runOpMode() {
        val drive = Drivebase(this)

        telemetry.addLine("Ready")
        telemetry.update()
        waitForStart()

        while (opModeIsActive()) {
            var y = -gamepad1.left_stick_y.toDouble()   // forward/back
            var x =  gamepad1.left_stick_x.toDouble()   // strafe
            var r =  gamepad1.right_stick_x.toDouble()  // rotate


            y = shape(deadband(y)); x = shape(deadband(x)); r = shape(deadband(r))

            val limit = if (gamepad1.right_bumper) 0.4 else 1.0  // slow mode
            drive.drive(y, x, r, limit)

            telemetry.addData("Mode", if (limit < 1.0) "SLOW" else "FULL")
            telemetry.addData("y/x/r", "%.2f / %.2f / %.2f", y, x, r)
            telemetry.update()
        }
        drive.stop()
    }

    private fun deadband(v: Double, th: Double = 0.05) = if (abs(v) < th) 0.0 else v
    private fun shape(v: Double) = sign(v) * abs(v).pow(3.0)
}
