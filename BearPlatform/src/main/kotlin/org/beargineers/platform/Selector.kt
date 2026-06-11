package org.beargineers.platform

class Selector(firstChoice: Choice, val onCompleted: (List<Pair<Choice, Option>>) -> Unit = {}) {
    class Option(val name: String, val nextChoice: Choice?, val f: () -> Unit)
    class Choice(val name: String, val options: List<Option>)
    private val answers = mutableListOf<Pair<Choice, Option>>()
    private var currentChoice: Choice? = firstChoice
    private var selectedOptionIndex = 0
    private var onCompletedHasBeenCalled = false

    fun RobotOpMode<*>.buttons() {
        button(gamepad1::a) {
            val choice = currentChoice
            if (choice != null) {
                val selectedOption = choice.options[selectedOptionIndex % choice.options.size]
                answers.add(choice to selectedOption)
                selectedOptionIndex = 0
                currentChoice = selectedOption.nextChoice
            }
        }

        button(gamepad1::dpad_down) {
            val choice = currentChoice
            if (choice != null) {
                selectedOptionIndex += 1
                if (selectedOptionIndex > choice.options.lastIndex) {
                    selectedOptionIndex = 0
                }
            }
        }

        button(gamepad1::dpad_up) {
            val choice = currentChoice
            if (choice != null) {
                selectedOptionIndex -= 1
                if (selectedOptionIndex < 0) {
                    selectedOptionIndex = choice.options.lastIndex
                }
            }
        }

        button(gamepad1::x) {
            if (currentChoice != null) {
                if (answers.lastOrNull()?.first == currentChoice) {
                    currentChoice = null
                }
            }
        }

        button(gamepad1::b) {
            if (currentChoice != null && answers.isNotEmpty()) {
                val (choice, _) = answers.removeAt(answers.lastIndex)
                currentChoice = choice
            }
        }
    }

    fun RobotOpMode<*>.update(): Boolean {
        val choice = currentChoice
        if (choice == null) {
            if (!onCompletedHasBeenCalled) {
                onCompleted(answers)
                onCompletedHasBeenCalled = true
            }
            return false
        }

        allButtons.forEach { it.update() }
        telemetry.addLine("dpad up/down, A select, B undo, X save")

        printSelectedAnswers()

        telemetry.addLine("Select ${choice.name}:")
        for ((i, option) in choice.options.withIndex()) {
            if (i == selectedOptionIndex) {
                telemetry.addLine("> " + option.name)
            } else {
                telemetry.addLine("  " + option.name)
            }
        }

        return true
    }
    
    private fun RobotOpMode<*>.printSelectedAnswers() {
        val (preliminary, final) = answers.partition { (choice, option) -> option.nextChoice != choice}

        for ((choice, option) in preliminary) {
            telemetry.addLine(choice.name + ": " + option.name)
        }

        if (final.isNotEmpty()) {
            telemetry.addLine(buildString {
                append(final.first().first.name + ": ")
                for ((_, option) in final) {
                    append(option.name.first())
                }
            })
        }

        telemetry.addLine("")
    }
}