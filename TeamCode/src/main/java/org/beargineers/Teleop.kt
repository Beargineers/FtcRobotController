package org.beargineers

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.beargineers.platform.Alliance
import org.beargineers.platform.BLUE_GOAL
import org.beargineers.platform.Position
import org.beargineers.platform.RED_GOAL
import org.beargineers.platform.RobotOpMode
import org.beargineers.robot.DecodeRobot
import org.beargineers.robot.IntakeMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

@Configurable
object TeleopConfigs {
    @JvmField
    var ROTATION_TRIGGER_REDUCTION: Float = 0.5f

    var POSITIONAL_GAIN = 60
    var ROTATIONAL_GAIN = 50
}

open class Driving(alliance: Alliance) : RobotOpMode<DecodeRobot>(alliance) {
    var fpvDrive = false

    var lookAtGoal = false

    override fun createRobot(opMode: RobotOpMode<DecodeRobot>): DecodeRobot {
        return DecodeRobot(this)
    }

    override fun bearInit() {
        super.bearInit()
/*
        toggleButton("Intake", gamepad1::x) { on ->
            intake.enable(if (on) IntakeMode.ON else IntakeMode.OFF)
        }
*/
        toggleButton("Intake Reverse", gamepad1::y) { on ->
            intake.enable(if (on) IntakeMode.REVERSE else IntakeMode.ON)
        }

        toggleButton("FPV Drive", gamepad1::b) {
            fpvDrive = it
        }
/*
        toggleButton("Shooter", gamepad1::a) { on ->
            shooter.enableFlywheel(on)
        }
*/
        toggleButton("Look At Goal", gamepad1::right_bumper) {
            lookAtGoal = it
        }

        button(gamepad1::a) {
            shooter.launch()
        }
    }

    override fun bearStart() {
        super.bearStart()

        robot.intake.enable(IntakeMode.ON)

        robot.shooter.enableFlywheel(true)
    }

    override fun bearLoop() {
        super.bearLoop()

        val slow = gamepad1.left_bumper
        telemetry.addData("Mode", if (slow) "SLOW" else "FULL")

        if (!fpvDrive) {
            val sign = if (alliance == Alliance.BLUE) -1 else 1

            val dx = gamepad1.left_stick_x.toDouble() * TeleopConfigs.POSITIONAL_GAIN * sign
            val dy = gamepad1.left_stick_y.toDouble() * TeleopConfigs.POSITIONAL_GAIN * -sign
            val rx = gamepad1.right_stick_x * sign
            val ry = gamepad1.right_stick_y * sign * -1
            val dh = 0.0 + (gamepad1.left_trigger - gamepad1.right_trigger) * TeleopConfigs.ROTATIONAL_GAIN

            val heading = Math.toDegrees(
                when {
                    lookAtGoal -> headingToGoal()
                    abs(ry) + abs(rx) > 0.01 -> atan2(ry.toDouble(), rx.toDouble())
                    else -> robot.currentPosition.heading
                }
            ) + dh

            val deltaPosition = Position(dx, dy, heading - Math.toDegrees(robot.currentPosition.heading),
                DistanceUnit.CM, AngleUnit.DEGREES)

            val targetPosition = robot.currentPosition.plus(deltaPosition)
            robot.driveToTarget(targetPosition, if (slow) 0.4 else 1.0)
        }
        else {
            val h = robot.currentPosition.toAngleUnit(AngleUnit.RADIANS).heading
            val forward = -gamepad1.left_stick_y.normalize()
            val strafe = gamepad1.left_stick_x.normalize()
            val dx = forward * cos(h) + strafe * sin(h)
            val dy = forward * sin(h) - strafe * cos(h)

            val rotation =
                (gamepad1.right_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger) * TeleopConfigs.ROTATION_TRIGGER_REDUCTION).normalize()
            val heading = Math.toDegrees(
                if (lookAtGoal) headingToGoal()
                else robot.currentPosition.heading + rotation
            )
            val deltaPosition = Position(dx, dy, heading - Math.toDegrees(robot.currentPosition.heading),
                DistanceUnit.CM, AngleUnit.DEGREES)

            val targetPosition = robot.currentPosition.plus(deltaPosition)
            robot.driveToTarget(targetPosition, if (slow) 0.4 else 1.0)
        }
    }

    private fun headingToGoal(): Double {
        val goal = (if (alliance == Alliance.BLUE) BLUE_GOAL else RED_GOAL).toUnit(DistanceUnit.CM)
        val cp = robot.currentPosition.toDistanceUnit(DistanceUnit.CM)
        val dx = goal.x - cp.x
        val dy = goal.y - cp.y
        return atan2(dy, dx)
    }

    fun Float.normalize(): Double = deadband(toDouble())

    private fun deadband(v: Double, th: Double = 0.01) = if (abs(v) < th) 0.0 else v
}

@TeleOp
class RedDriving() : Driving(Alliance.RED)
@TeleOp
class BlueDriving() : Driving(Alliance.BLUE)
