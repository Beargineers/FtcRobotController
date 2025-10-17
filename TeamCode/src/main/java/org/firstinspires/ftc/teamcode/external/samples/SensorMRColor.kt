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

import android.app.Activity
import android.graphics.Color
import android.view.View
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorSensor
import org.firstinspires.ftc.teamcode.internal.RobotOpModeBase

/**
 * This OpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The OpMode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Sensor: MR Color", group = "Sensor")
@Disabled
class SensorMRColor : RobotOpModeBase() {
    // Hardware Device Object
    val colorSensor: ColorSensor by hardware("sensor_color")

    // hsvValues is an array that will hold the hue, saturation, and value information.
    private val hsvValues = FloatArray(3)

    // The relativeLayout field is used to change the background color of the Robot Controller app
    private lateinit var relativeLayout: View

    // bPrevState and bCurrState represent the previous and current state of the button.
    private var bPrevState = false
    private var bCurrState = false

    // bLedOn represents the state of the LED.
    private var bLedOn = true

    override fun init() {
        // Get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        val relativeLayoutId = hardwareMap.appContext.resources.getIdentifier(
            "RelativeLayout", "id", hardwareMap.appContext.packageName
        )
        relativeLayout = (hardwareMap.appContext as Activity).findViewById(relativeLayoutId)

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn)

        telemetry.addData("Status", "Initialized")
    }

    override fun loop() {
        // Check the status of the x button on either gamepad.
        bCurrState = gamepad1.x

        // Check for button state transitions.
        if (bCurrState && (bCurrState != bPrevState)) {
            // Button is transitioning to a pressed state. So Toggle LED
            bLedOn = !bLedOn
            colorSensor.enableLed(bLedOn)
        }

        // Update previous state variable.
        bPrevState = bCurrState

        // Convert the RGB values to HSV values.
        Color.RGBToHSV(
            colorSensor.red() * 8,
            colorSensor.green() * 8,
            colorSensor.blue() * 8,
            hsvValues
        )

        // Send the info back to driver station using telemetry function.
        telemetry.addData("LED", if (bLedOn) "On" else "Off")
        telemetry.addData("Clear", colorSensor.alpha())
        telemetry.addData("Red  ", colorSensor.red())
        telemetry.addData("Green", colorSensor.green())
        telemetry.addData("Blue ", colorSensor.blue())
        telemetry.addData("Hue", hsvValues[0])

        // Change the background color to match the color detected by the RGB sensor.
        // Pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        relativeLayout.post {
            relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, hsvValues))
        }

        telemetry.update()
    }

    override fun stop() {
        // Set the panel back to the default color
        relativeLayout.post {
            relativeLayout.setBackgroundColor(Color.WHITE)
        }
    }
}
