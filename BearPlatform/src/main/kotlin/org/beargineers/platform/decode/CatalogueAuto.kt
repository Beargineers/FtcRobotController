package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.beargineers.platform.Alliance
import org.beargineers.platform.Selector
import org.beargineers.platform.Selector.Choice
import org.beargineers.platform.Selector.Option
import org.beargineers.platform.config

@Autonomous(group = "Construct")
class CatalogueAuto: ProgrammedAuto() {
    val ProgramCatalogue by config("F123")
    override lateinit var alliance: Alliance
    override lateinit var program: String

    private fun buildChoices(): Choice {
        val programOptions = ProgramCatalogue.split(',').map {
            Option(it.trim(), null) {
                program = it.trim()
            }
        }
        val frontPrograms = Choice("Program", programOptions.filter { it.name.startsWith('F') })
        val backPrograms = Choice("Program", programOptions.filter { it.name.startsWith('B') })

        val startingPointChoice = Choice("Starting Point", listOf(
            Option("Front", frontPrograms) {},
            Option("Back", backPrograms) {}
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

    val selector by lazy {
        Selector(buildChoices()) {
            for ((choice, option) in it) {
                option.f()
            }
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
            telemetry.addLine("Program '$program' selected. Ready to START")
        }
    }
}