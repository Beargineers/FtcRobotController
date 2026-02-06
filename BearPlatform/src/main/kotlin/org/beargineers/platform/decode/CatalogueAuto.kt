package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.beargineers.platform.Alliance
import org.beargineers.platform.Selector
import org.beargineers.platform.Selector.Choice
import org.beargineers.platform.Selector.Option
import org.beargineers.platform.config

data class Program(val name: String, val code: String)

@Autonomous(group = "Construct")
class CatalogueAuto: ProgrammedAuto() {
    val ProgramCatalogue by config("F123")
    override lateinit var alliance: Alliance
    override val program: String get() = selectedProgram.code
    lateinit var selectedProgram: Program


    private fun buildChoices(): Choice {
        val programs = ProgramCatalogue.split(',').map {it.trim()}.map {
            val (code, name) = if (it.contains(':')) it.split(':') else listOf(it, it)
            Program(name, code)
        }

        val programOptions = programs.map {
            Option("${it.name} (${it.code})", null) {
                selectedProgram = it
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
            telemetry.addLine("Program '${selectedProgram.name}' (${selectedProgram.code}) selected. Ready to START")
        }
    }
}