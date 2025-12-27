package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.beargineers.platform.Alliance
import org.beargineers.platform.Angle
import org.beargineers.platform.Position
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.closestPointInShootingZone
import org.beargineers.platform.cm
import org.beargineers.platform.config
import org.beargineers.platform.cos
import org.beargineers.platform.degrees
import org.beargineers.platform.headingToGoal
import org.beargineers.platform.radians
import org.beargineers.platform.shootingAngleCorrectionForMovement
import org.beargineers.platform.sin
import kotlin.math.abs
import kotlin.math.atan2

open class Driving(alliance: Alliance) : RobotOpMode<DecodeRobot>(alliance) {
    val ROTATION_TRIGGER_REDUCTION by robot.config(0.5)
    val POSITIONAL_GAIN by robot.config(60)
    val ROTATIONAL_GAIN by robot.config(50)
    var fpvDrive = false
    var lookAtGoal = false
    var goToShootZoneMode = false

    override fun bearInit() {
        super.bearInit()
/*
        toggleButton("Intake", gamepad1::x) { on ->
            robot.intakeMode(if (on) IntakeMode.ON else IntakeMode.OFF)
        }
*/
        toggleButton("Intake Reverse", gamepad1::y) { on ->
            robot.intakeMode(if (on) IntakeMode.REVERSE else IntakeMode.ON)
        }

        toggleButton("FPV Drive", gamepad1::b) {
            fpvDrive = it
            goToShootZoneMode = false
        }
/*
        toggleButton("Shooter", gamepad1::a) { on ->
            robot.enableFlywheel(on)
        }
*/
        toggleButton("Look At Goal", gamepad1::right_bumper) {
            lookAtGoal = it
            goToShootZoneMode = false
        }

        button(gamepad1::x){
            goToShootZoneMode = true
        }

        button(gamepad1::a) {
            robot.launch()
        }
    }

    override fun bearStart() {
        super.bearStart()

        robot.intakeMode(IntakeMode.ON)
        robot.enableFlywheel(true)
    }

    override fun bearLoop() {
        super.bearLoop()

        val slow = gamepad1.left_bumper
        telemetry.addData("Mode", if (slow) "SLOW" else "FULL")
        telemetry.addData("Going to goal", if(goToShootZoneMode) "YES" else "NO")
        // check if sticks are being touched
        if ((gamepad1.left_stick_x.toDouble() != 0.0) || (gamepad1.right_stick_x.toDouble() != 0.0) || (gamepad1.left_stick_y.toDouble() != 0.0) || (gamepad1.right_stick_y.toDouble() != 0.0)){
            goToShootZoneMode = false
        }
        if (goToShootZoneMode){
            robot.driveToTarget(robot.closestPointInShootingZone().withHeading(robot.headingToGoal()), 1.0)
        }else {
            if (!fpvDrive) {
                val sign = if (alliance == Alliance.BLUE) -1 else 1

                val dx = gamepad1.left_stick_x.toDouble() * POSITIONAL_GAIN * sign
                val dy = gamepad1.left_stick_y.toDouble() * POSITIONAL_GAIN * -sign
                val rx = gamepad1.right_stick_x * sign
                val ry = gamepad1.right_stick_y * sign * -1
                val dh = 0.0 + (gamepad1.left_trigger - gamepad1.right_trigger) * ROTATIONAL_GAIN

                val heading: Angle = when {
                    lookAtGoal -> robot.headingToGoal()
                    abs(ry) + abs(rx) > 0.01 -> atan2(ry.toDouble(), rx.toDouble()).radians
                    else -> robot.currentPosition.heading
                } + dh.degrees

                val deltaPosition = Position(dx.cm, dy.cm, heading - robot.currentPosition.heading)

                val targetPosition = robot.currentPosition.plus(deltaPosition)
                robot.driveToTarget(targetPosition, if (slow) 0.4 else 1.0)
            } else {
                val h = robot.currentPosition.heading - robot.shootingAngleCorrectionForMovement()
                val forward = -gamepad1.left_stick_y.normalize().cm
                val strafe = gamepad1.left_stick_x.normalize().cm
                val dx = (forward * cos(h) + strafe * sin(h)) * POSITIONAL_GAIN.toDouble()
                val dy = (forward * sin(h) - strafe * cos(h)) * POSITIONAL_GAIN.toDouble()

                val rotation =
                    (gamepad1.right_stick_x + (gamepad1.right_trigger - gamepad1.left_trigger) * ROTATION_TRIGGER_REDUCTION).normalize().degrees * 90.0
                val heading =
                    if (lookAtGoal) robot.headingToGoal() else robot.currentPosition.heading - rotation

                val deltaPosition = Position(dx, dy, heading - robot.currentPosition.heading)

                val targetPosition = robot.currentPosition.plus(deltaPosition)
                robot.driveToTarget(targetPosition, if (slow) 0.4 else 1.0)
            }
        }
    }





    fun Float.normalize(): Double = deadband(toDouble())
    fun Double.normalize(): Double = deadband(this)

    private fun deadband(v: Double, th: Double = 0.01) = if (abs(v) < th) 0.0 else v
}

@TeleOp
class RedDriving() : Driving(Alliance.RED)
@TeleOp
class BlueDriving() : Driving(Alliance.BLUE)
