package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.beargineers.platform.Alliance

@Autonomous(group = "Construct")
class ConstructAuto() : ProgrammedAuto() {
    override lateinit var alliance: Alliance

    override fun bearInit() {
        button(gamepad1::a) {
            if (runningActionSelection) {
                selectedActionsList.add(actionOptions[optionHighlight % actionOptions.size])
            } else if (selectedValueList.size < valueChoices.size) {
                selectedValueList.add(valueChoices[selectedValueList.size].list[optionHighlight % valueChoices[selectedValueList.size].list.size])
                if (selectedValueList.size == valueChoices.size){
                    runningActionSelection = true
                }
            }
        }
        button(gamepad1::dpad_down) {
            optionHighlight += 1
        }
        button(gamepad1::dpad_up) {
            optionHighlight -= 1
        }
        button(gamepad1::x) {
            if (runningActionSelection && selectedActionsList.isNotEmpty()) {
                runningActionSelection = false
            }
        }
        button(gamepad1::b) {
            if (selectedActionsList.isNotEmpty()) {
                selectedActionsList.removeAt(selectedActionsList.lastIndex)
            }else if (selectedValueList.isNotEmpty()){
                selectedValueList.removeAt(selectedValueList.lastIndex)
            }
        }

    }
    class Option(val name: String, val f: () -> Unit)

    class Choice(val name: String, val list: List<Option>)
    private val programBuilder = StringBuilder()
    override val program: String get() = programBuilder.toString()

    val valueChoices = listOf(
        Choice("Alliance", listOf(
            Option("Red"){
                alliance = Alliance.RED
            },
            Option("Blue"){
                alliance = Alliance.BLUE
            }
        )),

        Choice("Starting Point", listOf(
            Option("Front"){
                programBuilder.append('F')
            },
            Option("Back"){
                programBuilder.append('B')
            }
        )),
    )

    val actionOptions = listOf(
        Option("Collect 1") {
            programBuilder.append('1')
        },

        Option("Collect 2") {
            programBuilder.append('2')
        },

        Option("Collect 3") {
            programBuilder.append('3')
        },

        Option("Collect from the Box") {
            programBuilder.append('0')
        },

        Option("Collect from the Ramp") {
            programBuilder.append('4')
        },

        Option("Open Ramp") {
            programBuilder.append('R')
        },

        Option("Push Alliance Bot") {
            programBuilder.append('B')
        },

        Option("Shoot the load") {
            programBuilder.append("/")
        },

        Option("All next shootings from the Front") {
            programBuilder.append('F')
        },

        Option("All next shootings from the Back") {
            programBuilder.append('B')
        }
    )

    val selectedValueList = mutableListOf<Option>()
    val selectedActionsList = mutableListOf<Option>()
    var optionHighlight = 1
    var runningActionSelection = false

    fun Int.th(): String {
        return when ("$this".last().digitToInt()) {
            1 -> "${this}st"
            2 -> "${this}nd"
            3 -> "${this}rd"
            else -> "${this}th"
        }
    }

    override fun init_loop() {
        allButtons.forEach { it.update() }
        telemetry.addLine("dpad up/down, A select, B undo, X save program")
        // displays selections
        for ((i, choice) in valueChoices.withIndex()){
            telemetry.addData(choice.name, if (selectedValueList.size > i){selectedValueList[i].name}else{""})
        }
        for (i in 0..<selectedActionsList.size) {
            telemetry.addLine((i + 1).th() + ": " + selectedActionsList[i].name)
        }
        telemetry.addLine("")

        if (runningActionSelection) {   // displays action choices
            telemetry.addLine("Select Action:")
            for ((i, choice) in actionOptions.withIndex()) {
                if (i == (optionHighlight % actionOptions.size)) {
                    telemetry.addLine("> " + choice.name)
                } else {
                    telemetry.addLine("  " + choice.name)
                }
            }
        } else if (selectedValueList.size < valueChoices.size) {     // displays value choices
            telemetry.addLine("Please select the ${valueChoices[selectedValueList.size].name}:")
            for ((i, option) in valueChoices[selectedValueList.size].list.withIndex()) {
                if (i == (optionHighlight % valueChoices[selectedValueList.size].list.size)) {
                    telemetry.addLine("> " + option.name)
                } else {
                    telemetry.addLine("  " + option.name)
                }
            }
        } else {
            saveProgram()
            telemetry.addLine("Auto program is saved: $program")
        }
    }

    private var programIsSaved = false
    private fun saveProgram() {
        if (!programIsSaved) {
            programIsSaved = true
            for (choice in selectedValueList) {
                choice.f()
            }

            for (choice in selectedActionsList) {
                choice.f()
            }
        }
    }
}