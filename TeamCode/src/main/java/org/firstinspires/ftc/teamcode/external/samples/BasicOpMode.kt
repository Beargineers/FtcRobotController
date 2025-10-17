package org.firstinspires.ftc.teamcode.external.samples

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.internal.RobotOpModeBase

/**
 * Basic Iterative OpMode - Kotlin Implementation
 *
 * This OpMode executes a basic Tank Drive Teleop for a two wheeled robot.
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * The code demonstrates two drive modes:
 * - POV Mode: Left stick for forward/back, right stick for turning
 * - Tank Mode: Each stick controls one wheel (commented out by default)
 */
@TeleOp(name = "Basic: Iterative OpMode", group = "Iterative OpMode")
@Disabled
class BasicOpMode : RobotOpModeBase() {
    // Motors
    val leftDrive: DcMotor by hardware("left_drive")
    val rightDrive: DcMotor by hardware("right_drive")

    override fun init() {
        telemetry.addData("Status", "Initialized")

        // Configure motor directions
        leftDrive.direction = DcMotorSimple.Direction.REVERSE
        rightDrive.direction = DcMotorSimple.Direction.FORWARD

        telemetry.addData("Status", "Initialized")
    }

    override fun loop() {
        // POV Mode uses left stick to go forward, and right stick to turn
        // This uses basic math to combine motions and is easier to drive straight
        val drive = -gamepad1.left_stick_y.toDouble()
        val turn = gamepad1.right_stick_x.toDouble()
        val leftPower = Range.clip(drive + turn, -1.0, 1.0)
        val rightPower = Range.clip(drive - turn, -1.0, 1.0)

        // Tank Mode uses one stick to control each wheel
        // This requires no math, but it is hard to drive forward slowly and keep straight
        // val leftPower = -gamepad1.left_stick_y.toDouble()
        // val rightPower = -gamepad1.right_stick_y.toDouble()

        // Send calculated power to wheels
        leftDrive.power = leftPower
        rightDrive.power = rightPower

        // Show the elapsed game time and wheel power
        telemetry.addData("Status", "Run Time: $elapsed")
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower)
    }
}
