/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.internal.RobotOpModeBase
import kotlin.math.abs

// Calculate the COUNTS_PER_INCH for your specific drive train.
// Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
// For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
// For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
// This is gearing DOWN for less speed and more torque.
// For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
const val COUNTS_PER_MOTOR_REV = 1440.0    // eg: TETRIX Motor Encoder
const val DRIVE_GEAR_REDUCTION = 1.0       // No External Gearing.
const val WHEEL_DIAMETER_INCHES = 4.0      // For figuring circumference
const val COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415)
const val DRIVE_SPEED = 0.6
const val ENCODER_TURN_SPEED = 0.5

/**
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as an Iterative OpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a state machine that calls encoderDrive() to perform the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous(name = "Robot: Auto Drive By Encoder", group = "Robot")
@Disabled
class RobotAutoDriveByEncoder : RobotOpModeBase() {
    // Motors
    val leftDrive: DcMotor by hardware("left_drive")
    val rightDrive: DcMotor by hardware("right_drive")

    // State machine
    private lateinit var runtime: ElapsedTime
    private var currentStep = 0
    private var leftTarget = 0
    private var rightTarget = 0
    private var currentSpeed = 0.0
    private var currentTimeout = 0.0

    override fun init() {
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels. Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.direction = DcMotorSimple.Direction.REVERSE
        rightDrive.direction = DcMotorSimple.Direction.FORWARD

        leftDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        leftDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
            leftDrive.currentPosition,
            rightDrive.currentPosition)
        telemetry.update()
    }

    override fun start() {
        runtime = ElapsedTime()
        currentStep = 1
        // Step 1: Forward 48 Inches with 5 Sec timeout
        startEncoderDrive(DRIVE_SPEED, 48.0, 48.0, 5.0)
    }

    override fun loop() {
        when (currentStep) {
            1, 2, 3 -> {
                // Check if we're still moving
                if (runtime.seconds() < currentTimeout &&
                    leftDrive.isBusy && rightDrive.isBusy) {
                    // Display it for the driver
                    telemetry.addData("Running to", " %7d :%7d", leftTarget, rightTarget)
                    telemetry.addData("Currently at", " at %7d :%7d",
                        leftDrive.currentPosition,
                        rightDrive.currentPosition)
                } else {
                    // Stop all motion
                    leftDrive.power = 0.0
                    rightDrive.power = 0.0

                    // Turn off RUN_TO_POSITION
                    leftDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
                    rightDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER

                    // Move to next step
                    runtime.reset()
                    currentStep++

                    when (currentStep) {
                        2 -> startEncoderDrive(ENCODER_TURN_SPEED, 12.0, -12.0, 4.0)  // Turn Right 12 Inches with 4 Sec timeout
                        3 -> startEncoderDrive(DRIVE_SPEED, -24.0, -24.0, 4.0)  // Reverse 24 Inches with 4 Sec timeout
                        4 -> {
                            telemetry.addData("Path", "Complete")
                        }
                    }
                }
            }
        }

        telemetry.update()
    }

    override fun stop() {
        leftDrive.power = 0.0
        rightDrive.power = 0.0
    }

    /**
     * Method to start a relative move, based on encoder counts.
     * Encoders are not reset as the move is based on the current position.
     * Move will stop if any of three conditions occur:
     * 1) Move gets to the desired position
     * 2) Move runs out of time
     * 3) Driver stops the OpMode running.
     */
    private fun startEncoderDrive(speed: Double, leftInches: Double, rightInches: Double, timeoutS: Double) {
        // Determine new target position, and pass to motor controller
        leftTarget = leftDrive.currentPosition + (leftInches * COUNTS_PER_INCH).toInt()
        rightTarget = rightDrive.currentPosition + (rightInches * COUNTS_PER_INCH).toInt()
        leftDrive.targetPosition = leftTarget
        rightDrive.targetPosition = rightTarget

        // Turn On RUN_TO_POSITION
        leftDrive.mode = DcMotor.RunMode.RUN_TO_POSITION
        rightDrive.mode = DcMotor.RunMode.RUN_TO_POSITION

        // Reset the timeout time and start motion
        runtime.reset()
        leftDrive.power = abs(speed)
        rightDrive.power = abs(speed)

        // Store timeout for checking in loop
        currentSpeed = speed
        currentTimeout = timeoutS
    }
}
