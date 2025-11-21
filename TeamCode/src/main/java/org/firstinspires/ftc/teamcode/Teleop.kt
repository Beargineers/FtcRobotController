package org.firstinspires.ftc.teamcode
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.abs

@Configurable
object TeleopConfigs {
    @JvmField
    var ROTATION_TRIGGER_REDUCTION: Float = 0.5f
}

open class Driving(alliance: Alliance) : Robot(alliance) {
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

    val resetCoordsButton by lazy {
        Button(gamepad1::b).onRelease {
            currentPose = aprilTagPose ?: FIELD_CENTER
        }
    }

    override fun loop() {
        super.loop()
        launcherButton.update()
        intakeButton.update()
        feederButton.update()
        resetCoordsButton.update()

        val forward = -gamepad1.left_stick_y.normalize()
        val strafe =  gamepad1.left_stick_x.normalize()
        val rotation =  (gamepad1.right_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger) * TeleopConfigs.ROTATION_TRIGGER_REDUCTION).normalize()

        val slow = !gamepad1.left_bumper
        drive.drive(forward, strafe, rotation, slow)

        telemetry.addData("Mode", if (slow) "SLOW" else "FULL")
        telemetry.addData("f/s/r", "%.2f / %.2f / %.2f", forward, strafe, rotation)
    }

    fun Float.normalize(): Double = shape(deadband(this.toDouble()))

    private fun deadband(v: Double, th: Double = 0.01) = if (abs(v) < th) 0.0 else v
    private fun shape(v: Double) = v
}

@TeleOp class RedDriving() : Driving(Alliance.RED)
@TeleOp class BlueDriving() : Driving(Alliance.BLUE)
