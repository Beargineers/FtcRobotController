package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.beargineers.platform.Alliance
import org.beargineers.platform.Angle
import org.beargineers.platform.Frame
import org.beargineers.platform.Location
import org.beargineers.platform.Position
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.cm
import org.beargineers.platform.config
import org.beargineers.platform.cos
import org.beargineers.platform.degrees
import org.beargineers.platform.driveTo
import org.beargineers.platform.sin
import kotlin.math.abs

open class Driving(override val alliance: Alliance) : RobotOpMode<DecodeRobot>() {
    val POSITIONAL_GAIN by config(60)
    val ROTATIONAL_GAIN by config(20)

    val slowCoeff by config(0.4)
    private var fpvDrive = true
    private var lookAtGoal = false
    private var lookAtGoalBtnClickedAt = 0L

    override suspend fun DecodeRobot.autoProgram() {
        // Do nothing automatically
    }

    override fun bearInit() {
        super.bearInit()

        button(gamepad1::y) {
            robot.intakeMode = when (robot.intakeMode) {
                IntakeMode.OFF -> IntakeMode.ON
                IntakeMode.ON -> IntakeMode.OFF
                IntakeMode.REVERSE -> IntakeMode.ON
            }
        }.onHold {
            robot.intakeMode = IntakeMode.REVERSE
        }

/*
        toggleButton("FPV Drive", gamepad1::b) {
            fpvDrive = it
        }
*/

        button(gamepad1::a) {
            robot.flywheelEnabled = !robot.flywheelEnabled
        }

        button(gamepad1::right_stick_button) {
            lookAtGoal = true
            lookAtGoalBtnClickedAt = System.currentTimeMillis()
        }

        button({gamepad1.dpad_right || gamepad1.right_trigger > 0.1}) {
            lookAtGoal = true
            auto("Going to shooting zone") {
                goToShootingZoneAndShoot(if (alliance == Alliance.BLUE){ShootingZones.FRONT} else {
                    ShootingZones.BACK})
            }
        }

        button({gamepad1.dpad_left || gamepad1.left_trigger > 0.1}){
            lookAtGoal = true
            auto("Going to shooting zone") {
                goToShootingZoneAndShoot(if (alliance == Alliance.BLUE){ShootingZones.BACK} else {
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
            auto("Shooting") {
                robot.shoot()
            }
        }

        button(gamepad1::b) {
            auto("strafing to artifact") {
                collectArtifactsInView(false)
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

        button( gamepad1::x){
            robot.assumePosition(Position.zero(), 0.degrees)
        }
    }

    override fun bearStart() {
        super.bearStart()

        robot.assumePosition(lastKnownPosition, lastKnownTurretAngle)

/*
        robot.flywheelEnabled = true
        robot.intakeMode = IntakeMode.ON
*/
    }


    override fun bearLoop() {
        super.bearLoop()
        if (robot.clearForShooting()) {
            Frame.addData("Shooting", "Ready")
        }else{
            Frame.addData("Shooting", "NOT READY FOR SHOOTING")
        }
        val slow = gamepad1.left_bumper

        Frame.addData("Mode", if (slow) "SLOW" else "FULL")

        if (commandedRotation().degrees() != 0.0) {
            if (System.currentTimeMillis() - lookAtGoalBtnClickedAt > 500) {
                lookAtGoal = false
            }
        }

        Frame.addData("Goal locked", if (lookAtGoal) "YES" else "NO")

        val delta = if (!fpvDrive) {
            val sign = if (alliance == Alliance.BLUE) -1 else 1

            val dx = gamepad1.left_stick_x.toDouble() * POSITIONAL_GAIN * sign
            val dy = gamepad1.left_stick_y.toDouble() * POSITIONAL_GAIN * -sign

            Location(dx.cm, dy.cm)
        } else {
            val h = robot.currentPosition.heading
            val forward = -gamepad1.left_stick_y.normalize().cm
            val strafe = gamepad1.left_stick_x.normalize().cm
            val dx = (forward * cos(h) + strafe * sin(h)) * POSITIONAL_GAIN.toDouble()
            val dy = (forward * sin(h) - strafe * cos(h)) * POSITIONAL_GAIN.toDouble()

            Location(dx, dy)
        }

        val heading: Angle = commandedHeading()
        val deltaPosition = Position(delta.x, delta.y, heading - robot.currentPosition.heading)

        if (!isAutoActive()) {
            val targetPosition = robot.currentPosition.plus(deltaPosition)

            submitJob {
                robot.driveTo(targetPosition, if (slow) slowCoeff else 1.0, applyMirroring = false)
            }
        }
    }

    private fun commandedHeading(): Angle {
        val heading: Angle = when {
            lookAtGoal && !robot.hasTurret -> robot.headingToGoal()
            else -> robot.currentPosition.heading - commandedRotation()
        }
        return heading
    }

    private fun commandedRotation(): Angle {
        return (ROTATIONAL_GAIN * (gamepad1.right_stick_x +
                (0/*gamepad1.right_trigger - gamepad1.left_trigger*/) * 0.5).normalize()).degrees
    }


    fun Float.normalize(): Double = deadband(toDouble())
    fun Double.normalize(): Double = deadband(this)

    private fun deadband(v: Double, th: Double = 0.01) = if (abs(v) < th) 0.0 else v
}

@TeleOp
class RedDriving : Driving(Alliance.RED)
@TeleOp
class BlueDriving : Driving(Alliance.BLUE)
