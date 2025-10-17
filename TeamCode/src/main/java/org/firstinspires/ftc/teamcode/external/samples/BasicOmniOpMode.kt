package org.firstinspires.ftc.teamcode.external.samples

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.internal.RobotOpModeBase

/**
 * Basic Omni OpMode - Kotlin Iterative Implementation
 *
 * This OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 *
 * Holonomic drives provide the ability for the robot to move in three axes simultaneously:
 * 1) Axial:   Driving forward and backward          (Left-joystick Forward/Backward)
 * 2) Lateral: Strafing right and left                (Left-joystick Right and Left)
 * 3) Yaw:     Rotating clockwise and counter-clockwise (Right-joystick Right and Left)
 *
 * Note: It is critical to set the correct rotation direction for each motor.
 * When you first test your robot, if it moves backward when you push the left stick forward,
 * then you must flip the direction of all 4 motors.
 */
@TeleOp(name = "Basic: Omni OpMode", group = "Linear OpMode")
@Disabled
class BasicOmniOpMode : RobotOpModeBase() {
    // Motors
    val frontLeftDrive: DcMotor by hardware("front_left_drive")
    val backLeftDrive: DcMotor by hardware("back_left_drive")
    val frontRightDrive: DcMotor by hardware("front_right_drive")
    val backRightDrive: DcMotor by hardware("back_right_drive")

    override fun init() {
        // Most robots need the motors on one side to be reversed to drive forward
        // The motor reversals shown here are for a "direct drive" robot
        frontLeftDrive.direction = DcMotorSimple.Direction.REVERSE
        backLeftDrive.direction = DcMotorSimple.Direction.REVERSE
        frontRightDrive.direction = DcMotorSimple.Direction.FORWARD
        backRightDrive.direction = DcMotorSimple.Direction.FORWARD

        telemetry.addData("Status", "Initialized")
        telemetry.update()
    }

    override fun loop() {
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate
        val axial = -gamepad1.left_stick_y.toDouble()  // Note: pushing stick forward gives negative value
        val lateral = gamepad1.left_stick_x.toDouble()
        val yaw = gamepad1.right_stick_x.toDouble()

        // Combine the joystick requests for each axis-motion to determine each wheel's power
        var frontLeftPower = axial + lateral + yaw
        var frontRightPower = axial - lateral - yaw
        var backLeftPower = axial - lateral + yaw
        var backRightPower = axial + lateral - yaw

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion
        val max = maxOf(
            Math.abs(frontLeftPower),
            Math.abs(frontRightPower),
            Math.abs(backLeftPower),
            Math.abs(backRightPower)
        )

        if (max > 1.0) {
            frontLeftPower /= max
            frontRightPower /= max
            backLeftPower /= max
            backRightPower /= max
        }

        // Test code: Uncomment to test motor directions
        // Each button should make the corresponding motor run FORWARD
        // frontLeftPower = if (gamepad1.x) 1.0 else 0.0  // X gamepad
        // backLeftPower = if (gamepad1.a) 1.0 else 0.0   // A gamepad
        // frontRightPower = if (gamepad1.y) 1.0 else 0.0 // Y gamepad
        // backRightPower = if (gamepad1.b) 1.0 else 0.0  // B gamepad

        // Send calculated power to wheels
        frontLeftDrive.power = frontLeftPower
        frontRightDrive.power = frontRightPower
        backLeftDrive.power = backLeftPower
        backRightDrive.power = backRightPower

        // Show the elapsed game time and wheel power
        telemetry.addData("Status", "Run Time: $elapsed")
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower)
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower)
        telemetry.update()
    }
}
