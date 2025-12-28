package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.beargineers.platform.Button

@Autonomous
class ConstructAuto : OpMode() {

    private val allButtons = mutableListOf<Button>()

    fun button(test: () -> Boolean, callback: () -> Unit) {
        allButtons += Button(test).onRelease(callback)
    }

    override fun init() {
        button(gamepad1::a) {
            if (runningSelection) {
                selectedList.add(selectCategories[selectStep][optionHighlight])
                if (selectStep == selectCategories.size - 1) {
                    selectCategories.add(autoStepChoice)
                }
                selectStep += 1
                optionHighlight = 1
            }
        }
        button(gamepad1::dpad_right) {
            if (optionHighlight < selectCategories[selectStep].size - 1) {
                optionHighlight += 1
            }
        }
        button(gamepad1::dpad_left) {
            if (optionHighlight > 1) {
                optionHighlight -= 1
            }
        }
        button(gamepad1::x) {
            if (runningSelection) {
                runningSelection = false
            }
        }

    }

    override fun loop() {


    }

    val autoStepChoice = listOf(
        "Next step",
        "shoot close",
        "shoot far",
        "collect 1",
        "collect 2",
        "collect 3",
        "collect from box",
        "open the gate"
    )
    var selectCategories = mutableListOf(
        listOf("Alliance", "Blue", "Red"),
        listOf("Starting position", "far", "close")
    )

    val selectedList = mutableListOf<String>()
    var selectStep = 0
    var optionHighlight = 1

    var runningSelection = true

    override fun init_loop() {
        if (runningSelection) {
            super.init_loop()
            var selected = ""
            for (i in 0..<selectedList.size) {
                selected += selectedList[i] + ", "
            }

            telemetry.addData("Selections", selected)

            allButtons.forEach { it.update() }

            var selections = ""
            for (i in 1..<selectCategories[selectStep].size) {
                if (i == optionHighlight) {
                    selections += " __" + selectCategories[selectStep][i].uppercase() + "__ "
                } else {
                    selections += selectCategories[selectStep][i] + ", "
                }
            }
            telemetry.addData(selectCategories[selectStep][0], selections)
        } else {
            telemetry.addData("Auto Program", " Saved")
        }

    }
}