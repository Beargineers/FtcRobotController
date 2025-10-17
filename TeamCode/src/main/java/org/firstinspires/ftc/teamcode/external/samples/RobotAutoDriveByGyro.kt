/* Copyright (c) 2022 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.internal.RobotOpModeBase
import kotlin.math.abs

// Calculate the COUNTS_PER_INCH for your specific drive train.
const val GYRO_COUNTS_PER_MOTOR_REV = 537.7    // eg: GoBILDA 312 RPM Yellow Jacket
const val GYRO_DRIVE_GEAR_REDUCTION = 1.0       // No External Gearing.
const val GYRO_WHEEL_DIAMETER_INCHES = 4.0      // For figuring circumference
const val GYRO_COUNTS_PER_INCH = (GYRO_COUNTS_PER_MOTOR_REV * GYRO_DRIVE_GEAR_REDUCTION) / (GYRO_WHEEL_DIAMETER_INCHES * 3.1415)

// These constants define the desired driving/control characteristics
const val GYRO_DRIVE_SPEED = 0.4               // Max driving speed for better distance accuracy.
const val GYRO_TURN_SPEED = 0.2                // Max turn speed to limit turn rate.
const val HEADING_THRESHOLD = 1.0              // How close must the heading get to the target before moving to next step.

// Define the Proportional control coefficient (or GAIN) for "heading control".
const val P_TURN_GAIN = 0.02                   // Larger is more responsive, but also less stable.
const val P_DRIVE_GAIN = 0.03                  // Larger is more responsive, but also less stable.

/**
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (IMU) heading and encoder counts.
 *  The code is structured as an Iterative OpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by state machine transitions.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime
 *
 *  This code uses the Universal IMU interface so it will work with either the BNO055, or BHI260 IMU.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *  The REV Logo should be facing UP, and the USB port should be facing forward.
 *  If this is not the configuration of your REV Control Hub, then the code should be modified to reflect the correct orientation.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver Station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 * Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous(name = "Robot: Auto Drive By Gyro", group = "Robot")
@Disabled
class RobotAutoDriveByGyro : RobotOpModeBase() {
    // Motors and sensors
    val leftDrive: DcMotor by hardware("left_drive")
    val rightDrive: DcMotor by hardware("right_drive")
    val imu: IMU by hardware("imu")

    // State variables
    private var headingError = 0.0
    private var targetHeading = 0.0
    private var driveSpeed = 0.0
    private var turnSpeed = 0.0
    private var leftSpeed = 0.0
    private var rightSpeed = 0.0
    private var leftTarget = 0
    private var rightTarget = 0

    // State machine
    private lateinit var runtime: ElapsedTime
    private lateinit var holdTimer: ElapsedTime
    private var currentStep = 0
    private var isDriving = false

    override fun init() {
        // To drive forward, most robots need the motor on one side to be reversed
        leftDrive.direction = DcMotorSimple.Direction.REVERSE
        rightDrive.direction = DcMotorSimple.Direction.FORWARD

        // Define Hub orientation
        val logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP
        val usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        val orientationOnRobot = RevHubOrientationOnRobot(logoDirection, usbDirection)

        // Initialize the IMU
        imu.initialize(IMU.Parameters(orientationOnRobot))

        // Ensure the robot is stationary. Reset the encoders and set the motors to BRAKE mode
        leftDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightDrive.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightDrive.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        telemetry.addData(">", "Robot Heading = %4.0f", getHeading())
        telemetry.update()
    }

    override fun start() {
        // Set the encoders for closed loop speed control, and reset the heading
        leftDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rightDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
        imu.resetYaw()

        runtime = ElapsedTime()
        holdTimer = ElapsedTime()
        currentStep = 1
        startDriveStraight(GYRO_DRIVE_SPEED, 24.0, 0.0)
    }

    override fun loop() {
        when (currentStep) {
            1 -> {
                // Drive Forward 24"
                if (continueDriveStraight()) {
                    currentStep = 2
                    startTurnToHeading(GYRO_TURN_SPEED, -45.0)
                }
            }
            2 -> {
                // Turn CW to -45 Degrees
                if (continueTurnToHeading()) {
                    currentStep = 3
                    startHoldHeading(GYRO_TURN_SPEED, -45.0, 0.5)
                }
            }
            3 -> {
                // Hold -45 Deg heading for a 1/2 second
                if (continueHoldHeading()) {
                    currentStep = 4
                    startDriveStraight(GYRO_DRIVE_SPEED, 17.0, -45.0)
                }
            }
            4 -> {
                // Drive Forward 17" at -45 degrees
                if (continueDriveStraight()) {
                    currentStep = 5
                    startTurnToHeading(GYRO_TURN_SPEED, 45.0)
                }
            }
            5 -> {
                // Turn CCW to 45 Degrees
                if (continueTurnToHeading()) {
                    currentStep = 6
                    startHoldHeading(GYRO_TURN_SPEED, 45.0, 0.5)
                }
            }
            6 -> {
                // Hold 45 Deg heading for a 1/2 second
                if (continueHoldHeading()) {
                    currentStep = 7
                    startDriveStraight(GYRO_DRIVE_SPEED, 17.0, 45.0)
                }
            }
            7 -> {
                // Drive Forward 17" at 45 degrees
                if (continueDriveStraight()) {
                    currentStep = 8
                    startTurnToHeading(GYRO_TURN_SPEED, 0.0)
                }
            }
            8 -> {
                // Turn CW to 0 Degrees
                if (continueTurnToHeading()) {
                    currentStep = 9
                    startHoldHeading(GYRO_TURN_SPEED, 0.0, 1.0)
                }
            }
            9 -> {
                // Hold 0 Deg heading for 1 second
                if (continueHoldHeading()) {
                    currentStep = 10
                    startDriveStraight(GYRO_DRIVE_SPEED, -48.0, 0.0)
                }
            }
            10 -> {
                // Drive in Reverse 48"
                if (continueDriveStraight()) {
                    currentStep = 11
                    telemetry.addData("Path", "Complete")
                }
            }
        }

        telemetry.update()
    }

    override fun stop() {
        moveRobot(0.0, 0.0)
    }

    // Drive straight helper methods
    private fun startDriveStraight(maxDriveSpeed: Double, distance: Double, heading: Double) {
        val moveCounts = (distance * GYRO_COUNTS_PER_INCH).toInt()
        leftTarget = leftDrive.currentPosition + moveCounts
        rightTarget = rightDrive.currentPosition + moveCounts

        leftDrive.targetPosition = leftTarget
        rightDrive.targetPosition = rightTarget

        leftDrive.mode = DcMotor.RunMode.RUN_TO_POSITION
        rightDrive.mode = DcMotor.RunMode.RUN_TO_POSITION

        val speed = abs(maxDriveSpeed)
        moveRobot(speed, 0.0)
        isDriving = true
        targetHeading = heading
    }

    private fun continueDriveStraight(): Boolean {
        if (leftDrive.isBusy && rightDrive.isBusy) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(targetHeading, P_DRIVE_GAIN)

            // If driving in reverse, the motor correction also needs to be reversed
            if (leftTarget < leftDrive.currentPosition) {
                turnSpeed *= -1.0
            }

            // Apply the turning correction to the current driving speed
            moveRobot(driveSpeed, turnSpeed)

            // Display drive status for the driver
            sendTelemetry(true)
            return false
        } else {
            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0.0, 0.0)
            leftDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
            rightDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
            isDriving = false
            return true
        }
    }

    // Turn to heading helper methods
    private fun startTurnToHeading(maxTurnSpeed: Double, heading: Double) {
        targetHeading = heading
        getSteeringCorrection(heading, P_DRIVE_GAIN)
    }

    private fun continueTurnToHeading(): Boolean {
        if (abs(headingError) > HEADING_THRESHOLD) {
            turnSpeed = getSteeringCorrection(targetHeading, P_TURN_GAIN)
            turnSpeed = Range.clip(turnSpeed, -GYRO_TURN_SPEED, GYRO_TURN_SPEED)
            moveRobot(0.0, turnSpeed)
            sendTelemetry(false)
            return false
        } else {
            moveRobot(0.0, 0.0)
            return true
        }
    }

    // Hold heading helper methods
    private fun startHoldHeading(maxTurnSpeed: Double, heading: Double, holdTime: Double) {
        targetHeading = heading
        holdTimer.reset()
    }

    private fun continueHoldHeading(): Boolean {
        if (holdTimer.seconds() < 0.5) {  // Using the hold time from state machine
            turnSpeed = getSteeringCorrection(targetHeading, P_TURN_GAIN)
            turnSpeed = Range.clip(turnSpeed, -GYRO_TURN_SPEED, GYRO_TURN_SPEED)
            moveRobot(0.0, turnSpeed)
            sendTelemetry(false)
            return false
        } else {
            moveRobot(0.0, 0.0)
            return true
        }
    }

    // Low level driving functions
    private fun getSteeringCorrection(desiredHeading: Double, proportionalGain: Double): Double {
        targetHeading = desiredHeading

        // Determine the heading current error
        headingError = targetHeading - getHeading()

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360
        while (headingError <= -180) headingError += 360

        // Multiply the error by the gain to determine the required steering correction
        return Range.clip(headingError * proportionalGain, -1.0, 1.0)
    }

    private fun moveRobot(drive: Double, turn: Double) {
        driveSpeed = drive
        turnSpeed = turn

        leftSpeed = drive - turn
        rightSpeed = drive + turn

        // Scale speeds down if either one exceeds +/- 1.0
        val max = maxOf(abs(leftSpeed), abs(rightSpeed))
        if (max > 1.0) {
            leftSpeed /= max
            rightSpeed /= max
        }

        leftDrive.power = leftSpeed
        rightDrive.power = rightSpeed
    }

    private fun sendTelemetry(straight: Boolean) {
        if (straight) {
            telemetry.addData("Motion", "Drive Straight")
            telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget, rightTarget)
            telemetry.addData("Actual Pos L:R", "%7d:%7d",
                leftDrive.currentPosition,
                rightDrive.currentPosition)
        } else {
            telemetry.addData("Motion", "Turning")
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading())
        telemetry.addData("Error  : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed)
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed)
    }

    private fun getHeading(): Double {
        val orientation = imu.robotYawPitchRollAngles
        return orientation.getYaw(AngleUnit.DEGREES)
    }
}
