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
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.SwitchableLight
import org.firstinspires.ftc.teamcode.internal.RobotOpModeBase

const val WHITE_THRESHOLD = 0.5    // spans between 0.0 - 1.0 from dark to light
const val APPROACH_SPEED = 0.25

/**
 * This OpMode illustrates the concept of driving up to a line and then stopping.
 * The code is structured as an Iterative OpMode
 *
 * The Sensor used here can be a REV Color Sensor V2 or V3. Make sure the white LED is turned on.
 * The sensor can be plugged into any I2C port, and must be named "sensor_color" in the active configuration.
 *
 *   Depending on the height of your color sensor, you may want to set the sensor "gain".
 *   The higher the gain, the greater the reflected light reading will be.
 *   Use the SensorColor sample in this folder to determine the minimum gain value that provides an
 *   "Alpha" reading of 1.0 when you are on top of the white line. In this sample, we use a gain of 15
 *   which works well with a Rev V2 color sensor
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set halfway between the bare-tile, and white-line "Alpha" values.
 *   The reflected light value can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the sensor on and off the white line and note the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD halfway between the min and max.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Autonomous(name = "Robot: Auto Drive To Line", group = "Robot")
@Disabled
class RobotAutoDriveToLine : RobotOpModeBase() {
    // Motors
    val leftDrive: DcMotor by hardware("left_drive")
    val rightDrive: DcMotor by hardware("right_drive")

    // Color sensor
    val colorSensor: NormalizedColorSensor by hardware("sensor_color")

    // State
    private var isRunning = false

    override fun init() {
        // To drive forward, most robots need the motor on one side to be reversed
        leftDrive.direction = DcMotorSimple.Direction.REVERSE
        rightDrive.direction = DcMotorSimple.Direction.FORWARD

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER
        // rightDrive.mode = DcMotor.RunMode.RUN_USING_ENCODER

        // If necessary, turn ON the white LED (if there is no LED switch on the sensor)
        if (colorSensor is SwitchableLight) {
            (colorSensor as SwitchableLight).enableLight(true)
        }

        // Some sensors allow you to set your light sensor gain for optimal sensitivity...
        // See the SensorColor sample in this folder for how to determine the optimal gain.
        // A gain of 15 causes a Rev Color Sensor V2 to produce an Alpha value of 1.0 at about 1.5" above the floor.
        colorSensor.gain = 15f

        // Display the light level while we are waiting to start
        telemetry.addData("Status", "Ready to drive to white line.")
        getBrightness()
    }

    override fun start() {
        // Start the robot moving forward, and then begin looking for a white line
        leftDrive.power = APPROACH_SPEED
        rightDrive.power = APPROACH_SPEED
        isRunning = true
    }

    override fun loop() {
        if (isRunning) {
            val brightness = getBrightness()

            // Check if white line is seen
            if (brightness >= WHITE_THRESHOLD) {
                // Stop all motors
                leftDrive.power = 0.0
                rightDrive.power = 0.0
                isRunning = false
                telemetry.addData("Status", "White line detected!")
            }
        }

        telemetry.update()
    }

    override fun stop() {
        leftDrive.power = 0.0
        rightDrive.power = 0.0
    }

    // To obtain reflected light, read the normalized values from the color sensor. Return the Alpha channel.
    private fun getBrightness(): Double {
        val colors = colorSensor.normalizedColors
        telemetry.addData("Light Level (0 to 1)", "%4.2f", colors.alpha)
        return colors.alpha.toDouble()
    }
}
