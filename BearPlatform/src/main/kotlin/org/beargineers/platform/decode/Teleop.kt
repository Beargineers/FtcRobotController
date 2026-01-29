package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.beargineers.platform.Alliance
import org.beargineers.platform.Angle
import org.beargineers.platform.Position
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.cm
import org.beargineers.platform.config
import org.beargineers.platform.cos
import org.beargineers.platform.degrees
import org.beargineers.platform.sin
import kotlin.math.abs

open class Driving(alliance: Alliance) : RobotOpMode<DecodeRobot>(alliance) {
    val POSITIONAL_GAIN by config(60)
    val ROTATIONAL_GAIN by config(20)

    val slowCoeff by config(0.4)
    private var fpvDrive = true
    private var lookAtGoal = false
    private var lookAtGoalBtnClickedAt = 0L

    override fun bearInit() {
        super.bearInit()

        button(gamepad1::y) {
            robot.intakeMode(when (robot.intakeMode) {
                IntakeMode.OFF -> IntakeMode.ON
                IntakeMode.ON -> IntakeMode.OFF
                IntakeMode.REVERSE -> IntakeMode.ON
            })
        }.onHold {
            robot.intakeMode(IntakeMode.REVERSE)
        }

/*
        toggleButton("FPV Drive", gamepad1::b) {
            fpvDrive = it
        }
*/

        toggleButton("Shooter", gamepad1::a){
            robot.enableFlywheel(it)
        }

        button(gamepad1::right_stick_button) {
            lookAtGoal = true
            lookAtGoalBtnClickedAt = System.currentTimeMillis()
        }

        button(gamepad1::dpad_right) {
            lookAtGoal = true
            auto("Going to shooting zone") {
                goToShootingZoneAndShoot(if (opMode.alliance == Alliance.BLUE){ShootingZones.FRONT} else {
                    ShootingZones.BACK})
            }
        }

        button(gamepad1::dpad_left){
            lookAtGoal = true
            auto("Going to shooting zone") {
                goToShootingZoneAndShoot(if (opMode.alliance == Alliance.BLUE){ShootingZones.BACK} else {
                    ShootingZones.FRONT})
            }
        }

        button(gamepad1::dpad_up) {
            lookAtGoal = false
            auto("Going to open the ramp") {
                if (gamepad1.left_bumper) {
                    openRampAndCollect()
                }
                else {
                    openRamp()
                }
            }
        }

        button(gamepad1::dpad_down) {
            lookAtGoal = false
            auto("Going to the base") {
                park()
            }
        }

        button(gamepad1::right_bumper) {
            val position = robot.currentPosition
            robot.launch()
            auto("Holding position") {
                holdPositionLookAtGoal(position.location())
            }
        }

        button( gamepad2::a){
            lookAtGoal = false
            auto("Going to cursor"){
                goToCursorLocation()
            }
        }

        button(gamepad2::dpad_up) {
            robot.adjustShooting(+0.01, 0.0)
        }

        button(gamepad2::dpad_down) {
            robot.adjustShooting(-0.01, 0.0)
        }

        button(gamepad2::dpad_left) {
            robot.adjustShooting(+0.0, 0.33)
        }

        button(gamepad2::dpad_right) {
            robot.adjustShooting(+0.0, -0.33)
        }

        button(gamepad1::x){
            robot.getReadyForShoot()
        }
    }

    override fun bearStart() {
        super.bearStart()
        robot.enableFlywheel(true)

        robot.assumePosition(lastKnownPosition)

/*
        robot.intakeMode(IntakeMode.ON)
*/
    }


    override fun bearLoop() {
        super.bearLoop()
        if (robot.clearForShooting()) {
            telemetry.addData("", "Can shoot")
        }else{
            telemetry.addData("", "NOT READY FOR SHOOTING")
        }
        val slow = gamepad1.left_bumper

        telemetry.addData("Mode", if (slow) "SLOW" else "FULL")

        if (commandedRotation().degrees() != 0.0) {
            if (System.currentTimeMillis() - lookAtGoalBtnClickedAt > 500) {
                lookAtGoal = false
            }
        }

        telemetry.addData("Goal locked", if (lookAtGoal) "YES" else "NO")

        if (!fpvDrive) {
            val sign = if (alliance == Alliance.BLUE) -1 else 1

            val dx = gamepad1.left_stick_x.toDouble() * POSITIONAL_GAIN * sign
            val dy = gamepad1.left_stick_y.toDouble() * POSITIONAL_GAIN * -sign

            val heading: Angle = commandedHeading()

            val deltaPosition = Position(dx.cm, dy.cm, heading - robot.currentPosition.heading)

            val targetPosition = robot.currentPosition.plus(deltaPosition)
            robot.targetSpeed = if (slow) slowCoeff else 1.0
            robot.driveToTarget(targetPosition)
        } else {
            val h = robot.currentPosition.heading - robot.shootingAngleCorrectionForMovement()
            val forward = -gamepad1.left_stick_y.normalize().cm
            val strafe = gamepad1.left_stick_x.normalize().cm
            val dx = (forward * cos(h) + strafe * sin(h)) * POSITIONAL_GAIN.toDouble()
            val dy = (forward * sin(h) - strafe * cos(h)) * POSITIONAL_GAIN.toDouble()

            val heading: Angle = commandedHeading()

            val deltaPosition = Position(dx, dy, heading - robot.currentPosition.heading)

            val targetPosition = robot.currentPosition.plus(deltaPosition)
            robot.targetSpeed = if (slow) slowCoeff else 1.0
            robot.driveToTarget(targetPosition)
        }
    }

    private fun commandedHeading(): Angle {
        val heading: Angle = when {
            lookAtGoal -> robot.headingToGoal()
            else -> robot.currentPosition.heading - commandedRotation()
        }
        return heading
    }

    private fun commandedRotation(): Angle {
        return (ROTATIONAL_GAIN * (gamepad1.right_stick_x +
                (gamepad1.right_trigger - gamepad1.left_trigger) * 0.5).normalize()).degrees
    }


    fun Float.normalize(): Double = deadband(toDouble())
    fun Double.normalize(): Double = deadband(this)

    private fun deadband(v: Double, th: Double = 0.01) = if (abs(v) < th) 0.0 else v
}

@TeleOp
class RedDriving() : Driving(Alliance.RED)
@TeleOp
class BlueDriving() : Driving(Alliance.BLUE)
