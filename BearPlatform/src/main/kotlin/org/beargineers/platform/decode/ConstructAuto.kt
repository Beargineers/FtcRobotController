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

        action("Collect 1", '1')
        action("Collect 2", '2')
        action("Collect 3", '3')
        action("Collect from the Box", '0')
        action("Collect from the Ramp", '4')
        action("Open Ramp", 'R')
        action("Push Alliance Bot", 'B')
        action("Shoot the Load", '/')
        action("All next shootings from the Front", 'F')
        action("All next shootings from the Back", 'B')

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

    override fun init_loop() {
        val active = with(selector) {
            update()
        }
        if (!active) {
            telemetry.addLine("Program '$program' is saved. Ready to START")
        }
    }
}