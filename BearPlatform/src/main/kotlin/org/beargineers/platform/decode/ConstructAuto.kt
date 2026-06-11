package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.beargineers.platform.Alliance
import org.beargineers.platform.Selector
import org.beargineers.platform.Selector.Choice
import org.beargineers.platform.Selector.Option

@Autonomous(group = "Construct")
class ConstructAuto() : ProgrammedAuto() {
    override lateinit var alliance: Alliance
    private val programBuilder = StringBuilder()
    override val program: String get() = programBuilder.toString()

    private fun buildChoices(): Choice {
        val actions = mutableListOf<Option>()
        val actionChoice = Choice("Actions", actions)

        fun action(name: String, code: Char) {
            actions.add(Option(name, actionChoice) {
                programBuilder.append(code)
            })
        }

        action("1 Collect 1", '1')
        action("2 Collect 2", '2')
        action("3 Collect 3", '3')
        action("0 Collect from the Box", '0')
        action("V Collect from the Box Visually", 'V')
        action("4 Collect from the Ramp", '4')
        action("R Open Ramp", 'R')
        action("P Push Alliance Bot", 'P')
        action("/ Shoot the Load", '/')
        action("F All next shootings from the Front", 'F')
        action("B All next shootings from the Back", 'B')

        val startingPointChoice = Choice("Starting Point", listOf(
            Option("Front", actionChoice) {
                programBuilder.append('F')
            },
            Option("Back", actionChoice) {
                programBuilder.append('B')
            }
        ))

        val allianceChoice = Choice("Alliance", listOf(
            Option("Red", startingPointChoice) {
                alliance = Alliance.RED
            },
            Option("Blue", startingPointChoice) {
                alliance = Alliance.BLUE
            }
        ))

        return allianceChoice
    }

    private val selector = Selector(buildChoices()) {
        for ((choice, option) in it) {
            option.f()
        }
    }

    override fun bearInit() {
        with(selector) {
            buttons()
        }
    }

    override fun bearInitLoop() {
        val active = with(selector) {
            update()
        }
        if (!active) {
            telemetry.addLine("Program '$program' is saved. Ready to START")
        }
    }
}