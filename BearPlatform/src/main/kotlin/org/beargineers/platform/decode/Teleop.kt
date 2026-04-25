package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.beargineers.platform.Alliance
import org.beargineers.platform.Angle
import org.beargineers.platform.Frame
import org.beargineers.platform.Location
import org.beargineers.platform.Position
import org.beargineers.platform.RobotDimensions
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.cm
import org.beargineers.platform.config
import org.beargineers.platform.cos
import org.beargineers.platform.degrees
import org.beargineers.platform.driveTo
import org.beargineers.platform.sin
import org.beargineers.platform.tilePosition
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

    private fun Gamepad.firstDriverControls() {
        button(::a) {
            robot.intakeMode = when (robot.intakeMode) {
                IntakeMode.OFF -> IntakeMode.ON
                IntakeMode.ON -> IntakeMode.OFF
                IntakeMode.REVERSE -> IntakeMode.ON
            }
        }.onHold {
            robot.intakeMode = IntakeMode.REVERSE
        }

        button(::left_stick_button) {
            if (isDevMode()) {
                robot.flywheelEnabled = !robot.flywheelEnabled
            }
        }

        button(::right_stick_button) {
            lookAtGoal = true
            lookAtGoalBtnClickedAt = System.currentTimeMillis()
        }

        button(::right_bumper) {
            lookAtGoal = true
            auto("Going to shooting zone") {
                goToShootingZoneAndShoot(if (alliance == Alliance.BLUE){ShootingZones.FRONT} else {
                    ShootingZones.BACK})
            }
        }

        button(::left_bumper) {
            lookAtGoal = true
            auto("Going to shooting zone") {
                goToShootingZoneAndShoot(if (alliance == Alliance.BLUE){ShootingZones.BACK} else {
                    ShootingZones.FRONT})
            }
        }

        button({ right_trigger > 0.1}) {
            lookAtGoal = false
            auto("Going to closest zone") {
                goToShootingZoneAndShoot(ShootingZones.CLOSEST)
            }
        }

        button({ left_trigger > 0.1 }) {
            auto("Collect by vision") {
                collectArtifactsInView(false)
            }
        }

        button(::y) {
            lookAtGoal = false
            auto("Collect from ramp") {
                openRampAndCollect()
            }
        }

        button(::b) {
            lookAtGoal = false
            auto("Open ramp") {
                openRamp()
            }
        }

        button(::dpad_down) {
            lookAtGoal = false
            auto("Going to the base") {
                park()
            }
        }

        button( ::x) {
            robot.assumePosition(Position.ZERO)
            robot.resetTurret()
        }
    }

    private fun Gamepad.secondDriverControls() {
        button(::dpad_up) {
            robot.adjustShooting(+0.01, 0.0)
        }

        button(::dpad_down) {
            robot.adjustShooting(-0.01, 0.0)
        }

        button(::dpad_left) {
            robot.adjustShooting(+0.0, 0.33)
        }

        button(::dpad_right) {
            robot.adjustShooting(+0.0, -0.33)
        }
    }

    override fun bearInit() {
        gamepad1.firstDriverControls()
        gamepad2.secondDriverControls()
    }

    override fun bearStart() {
        super.bearStart()

        if (lastKnownPosition.isNA()) {
            error("Position is not initialized. Cannot start")
        }

        robot.assumePosition(lastKnownPosition)

        if (!isDevMode()) {
            robot.flywheelEnabled = true
            robot.intakeMode = IntakeMode.ON
        }
    }

    override fun bearInitLoop() {
        Frame.addData("Position", if (lastKnownPosition.isNA()) "NOT INITIALIZED!" else lastKnownPosition)
        Frame.addLine("[X] for back corner on BLUE side")
        Frame.addLine("[B] for back corner on RED side")
        Frame.addLine("[Y] for center facing goals wall")
        Frame.addLine("[A] for center facing back wall")
        Frame.addLine("Turret must be aligned with robot's heading in all cases")

        if (gamepad1.aWasPressed()) {
            robot.assumePosition(Position.ZERO)
            robot.resetTurret()
        }

        if (gamepad1.yWasPressed()) {
            robot.assumePosition(Position.ZERO.rotate(180.degrees))
            robot.resetTurret()
        }

        if (gamepad1.xWasPressed()) {
            val pos = tilePosition("A1BL:180").shift(-RobotDimensions.ROBOT_BACK_OFFSET, (RobotDimensions.ROBOT_WIDTH / 2))
            robot.assumePosition(pos)
            robot.resetTurret()
        }

        if (gamepad1.bWasPressed()) {
            val pos = tilePosition("F1BR:180").shift(-RobotDimensions.ROBOT_BACK_OFFSET, -(RobotDimensions.ROBOT_WIDTH / 2))
            robot.assumePosition(pos)
            robot.resetTurret()
        }
    }

    override fun bearLoop() {
        super.bearLoop()
        Frame.addData("Shooting", robot.clearForShooting() ?: "Ready")

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

        if (!deltaPosition.isZero()) {
            cancelAuto()
        }

        if (!isAutoActive()) {
            val targetPosition = robot.currentPosition.plus(deltaPosition)

            submitJob("!Driving") {
                val slow = false
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
