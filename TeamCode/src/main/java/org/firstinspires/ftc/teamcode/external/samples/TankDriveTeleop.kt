package org.firstinspires.ftc.teamcode.external.samples

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.internal.RobotOpModeBase

const val MID_SERVO = 0.5
const val CLAW_SPEED = 0.02
const val ARM_UP_POWER = 0.50
const val ARM_DOWN_POWER = -0.25

/**
 * Tank Drive TeleOp - Kotlin Implementation
 *
 * This OpMode executes a Tank Drive control where the left and right joysticks
 * control the left and right motors respectively.
 *
 * Controls:
 * - Left stick Y: Controls left drive motor
 * - Right stick Y: Controls right drive motor
 * - Y button: Raises arm
 * - A button: Lowers arm
 * - Right bumper: Opens claw
 * - Left bumper: Closes claw
 */
@TeleOp(name = "Robot: Tank Drive", group = "Robot")
@Disabled
class TankDriveTeleop : RobotOpModeBase() {
    // Motors
    val leftDrive: DcMotor by hardware("left_drive")
    val rightDrive: DcMotor by hardware("right_drive")
    val leftArm: DcMotor by hardware("left_arm")

    // Servos
    val leftClaw: Servo by hardware("left_hand")
    val rightClaw: Servo by hardware("right_hand")

    // State
    var clawOffset = 0.0

    override fun init() {
        // Configure motor directions
        leftDrive.direction = DcMotorSimple.Direction.REVERSE
        rightDrive.direction = DcMotorSimple.Direction.FORWARD

        // Initialize servos to middle position
        leftClaw.position = MID_SERVO
        rightClaw.position = MID_SERVO

        telemetry.addData(">", "Robot Ready. Press START.")
    }

    override fun loop() {
        // Tank drive - joysticks are negative when pushed forward
        val left = -gamepad1.left_stick_y.toDouble()
        val right = -gamepad1.right_stick_y.toDouble()
        leftDrive.power = left
        rightDrive.power = right

        // Update claw position based on bumpers
        if (gamepad1.right_bumper) {
            clawOffset += CLAW_SPEED
        } else if (gamepad1.left_bumper) {
            clawOffset -= CLAW_SPEED
        }
        clawOffset = Range.clip(clawOffset, -0.5, 0.5)
        leftClaw.position = MID_SERVO + clawOffset
        rightClaw.position = MID_SERVO - clawOffset

        // Update arm position based on Y and A buttons
        leftArm.power = when {
            gamepad1.y -> ARM_UP_POWER
            gamepad1.a -> ARM_DOWN_POWER
            else -> 0.0
        }

        // Send telemetry
        telemetry.addData("claw", "Offset = %.2f", clawOffset)
        telemetry.addData("left", "%.2f", left)
        telemetry.addData("right", "%.2f", right)
    }
}
