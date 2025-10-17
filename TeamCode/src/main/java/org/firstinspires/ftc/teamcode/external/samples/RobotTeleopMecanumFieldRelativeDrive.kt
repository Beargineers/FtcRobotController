/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.external.samples

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.internal.RobotOpModeBase
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin

const val MAX_SPEED = 1.0  // make this slower for outreaches

/**
 * This OpMode illustrates how to program your robot to drive field relative. This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Robot: Field Relative Mecanum Drive", group = "Robot")
@Disabled
class RobotTeleopMecanumFieldRelativeDrive : RobotOpModeBase() {
    // Motors
    val frontLeftDrive: DcMotor by hardware("front_left_drive")
    val frontRightDrive: DcMotor by hardware("front_right_drive")
    val backLeftDrive: DcMotor by hardware("back_left_drive")
    val backRightDrive: DcMotor by hardware("back_right_drive")

    // IMU to get the current direction the robot is facing
    val imu: IMU by hardware("imu")

    override fun init() {
        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.direction = DcMotorSimple.Direction.REVERSE
        frontLeftDrive.direction = DcMotorSimple.Direction.REVERSE

        // This uses RUN_USING_ENCODER to be more accurate. If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
        frontRightDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
        backLeftDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
        backRightDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER

        // This needs to be changed to match the orientation on your robot
        val logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP
        val usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD

        val orientationOnRobot = RevHubOrientationOnRobot(logoDirection, usbDirection)
        imu.initialize(IMU.Parameters(orientationOnRobot))
    }

    override fun loop() {
        telemetry.addLine("Press A to reset Yaw")
        telemetry.addLine("Hold left bumper to drive in robot relative")
        telemetry.addLine("The left joystick sets the robot direction")
        telemetry.addLine("Moving the right joystick left and right turns the robot")

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.a) {
            imu.resetYaw()
        }

        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y.toDouble(), gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x.toDouble())
        } else {
            driveFieldRelative(-gamepad1.left_stick_y.toDouble(), gamepad1.left_stick_x.toDouble(), gamepad1.right_stick_x.toDouble())
        }
    }

    // This routine drives the robot field relative
    private fun driveFieldRelative(forward: Double, right: Double, rotate: Double) {
        // First, convert direction being asked to drive to polar coordinates
        var theta = atan2(forward, right)
        val r = hypot(right, forward)

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta - imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS))

        // Third, convert back to cartesian
        val newForward = r * sin(theta)
        val newRight = r * cos(theta)

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate)
    }

    // Thanks to FTC16072 for sharing this code!!
    private fun drive(forward: Double, right: Double, rotate: Double) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        val frontLeftPower = forward + right + rotate
        val frontRightPower = forward - right - rotate
        val backRightPower = forward + right - rotate
        val backLeftPower = forward - right + rotate

        var maxPower = 1.0

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = max(maxPower, abs(frontLeftPower))
        maxPower = max(maxPower, abs(frontRightPower))
        maxPower = max(maxPower, abs(backRightPower))
        maxPower = max(maxPower, abs(backLeftPower))

        // We multiply by MAX_SPEED so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full speed.
        frontLeftDrive.power = MAX_SPEED * (frontLeftPower / maxPower)
        frontRightDrive.power = MAX_SPEED * (frontRightPower / maxPower)
        backLeftDrive.power = MAX_SPEED * (backLeftPower / maxPower)
        backRightDrive.power = MAX_SPEED * (backRightPower / maxPower)
    }
}
