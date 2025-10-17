package org.firstinspires.ftc.teamcode.external.samples

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.internal.RobotOpModeBase

const val POV_MID_SERVO = 0.5
const val POV_CLAW_SPEED = 0.02
const val POV_ARM_UP_POWER = 0.45
const val POV_ARM_DOWN_POWER = -0.45

/**
 * Robot Teleop POV - Kotlin Iterative Implementation
 *
 * This OpMode executes a POV Game style Teleop for a direct drive robot.
 *
 * In this mode the left stick moves the robot FWD and back, the right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 */
@TeleOp(name = "Robot: Teleop POV", group = "Robot")
@Disabled
class RobotTeleopPOV : RobotOpModeBase() {
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
        leftClaw.position = POV_MID_SERVO
        rightClaw.position = POV_MID_SERVO

        telemetry.addData(">", "Robot Ready. Press START.")
        telemetry.update()
    }

    override fun loop() {
        // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right
        val drive = -gamepad1.left_stick_y.toDouble()
        val turn = gamepad1.right_stick_x.toDouble()

        // Combine drive and turn for blended motion
        var left = drive + turn
        var right = drive - turn

        // Normalize the values so neither exceed +/- 1.0
        val max = maxOf(Math.abs(left), Math.abs(right))
        if (max > 1.0) {
            left /= max
            right /= max
        }

        // Output the safe values to the motor drives
        leftDrive.power = left
        rightDrive.power = right

        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper) {
            clawOffset += POV_CLAW_SPEED
        } else if (gamepad1.left_bumper) {
            clawOffset -= POV_CLAW_SPEED
        }

        // Move both servos to new position. Assume servos are mirror image of each other
        clawOffset = Range.clip(clawOffset, -0.5, 0.5)
        leftClaw.position = POV_MID_SERVO + clawOffset
        rightClaw.position = POV_MID_SERVO - clawOffset

        // Use gamepad buttons to move arm up (Y) and down (A)
        leftArm.power = when {
            gamepad1.y -> POV_ARM_UP_POWER
            gamepad1.a -> POV_ARM_DOWN_POWER
            else -> 0.0
        }

        // Send telemetry message to signify robot running
        telemetry.addData("claw", "Offset = %.2f", clawOffset)
        telemetry.addData("left", "%.2f", left)
        telemetry.addData("right", "%.2f", right)
        telemetry.update()
    }
}
