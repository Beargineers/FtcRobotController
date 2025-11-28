package org.beargineers

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.beargineers.platform.Alliance
import org.beargineers.platform.FIELD_CENTER
import kotlin.math.abs

@Configurable
object TeleopConfigs {
    @JvmField
    var ROTATION_TRIGGER_REDUCTION: Float = 0.5f
}

open class Driving(alliance: Alliance) : DecodeOpMode(alliance) {
    override fun bearInit() {
        super.bearInit()

        toggleButton("Intake", gamepad1::x) { on ->
            intake.enable(if (on) IntakeMode.ON else IntakeMode.OFF)
        }

        toggleButton("Intake Reverse", gamepad1::y) { on ->
            intake.enable(if (on) IntakeMode.REVERSE else IntakeMode.OFF)
        }

        toggleButton("Shooter", gamepad1::a) { on ->
            shooter.enableFlywheel(on)
        }

        button(gamepad1::right_bumper) {
            shooter.launch()
        }

        button(gamepad1::b) {
            currentPosition = aprilTagPose ?: FIELD_CENTER
        }
    }

    override fun bearLoop() {
        super.bearLoop()

        val forward = -gamepad1.left_stick_y.normalize()
        val strafe = gamepad1.left_stick_x.normalize()
        val rotation =
            (gamepad1.right_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger) * TeleopConfigs.ROTATION_TRIGGER_REDUCTION).normalize()

        val slow = !gamepad1.left_bumper
        robot.drive.drive(forward, strafe, rotation, slow)

        telemetry.addData("Mode", if (slow) "SLOW" else "FULL")
        telemetry.addData("f/s/r", "%.2f / %.2f / %.2f", forward, strafe, rotation)
    }

    fun Float.normalize(): Double = shape(deadband(this.toDouble()))

    private fun deadband(v: Double, th: Double = 0.01) = if (abs(v) < th) 0.0 else v
    private fun shape(v: Double) = v
}

@TeleOp
class RedDriving() : Driving(Alliance.RED)
@TeleOp
class BlueDriving() : Driving(Alliance.BLUE)
