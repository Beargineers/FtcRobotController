package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.abs

@TeleOp(name = "Mecanum Drive Kotlin", group = "Drive")
class MecanumTeleOp : Robot() {
    val intakeButton by lazy {
        ToggleButton(gamepad1::x) { on ->
            telemetry.addLine("Intake running: $on")
            intake.enable(on)
        }
    }

    val launcherButton by lazy {
        ToggleButton(gamepad1::a) { on ->
            telemetry.addLine("Launcher flywheel running: $on")
            shooter.enableFlywheel(on)
        }
    }

    val feederButton by lazy {
        Button(gamepad1::right_bumper).onRelease {
            shooter.launch()
        }
    }

    override fun loop() {
        super.loop()
        launcherButton.update()
        intakeButton.update()
        feederButton.update()

        val y = -gamepad1.left_stick_y.normalize()   // forward/back
        val x =  gamepad1.left_stick_x.normalize()  // strafe
        val r =  (gamepad1.right_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger) / 3).normalize() // rotate

        val slow = !gamepad1.left_bumper
        drive.drive(y, x, r, slow)

        telemetry.addData("Mode", if (slow) "SLOW" else "FULL")
        telemetry.addData("y/x/r", "%.2f / %.2f / %.2f", y, x, r)
    }

    fun Float.normalize(): Double = shape(deadband(this.toDouble()))

    private fun deadband(v: Double, th: Double = 0.01) = if (abs(v) < th) 0.0 else v
    private fun shape(v: Double) = v
}
