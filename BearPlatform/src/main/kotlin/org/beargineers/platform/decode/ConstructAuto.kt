package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.beargineers.platform.Alliance
import org.beargineers.platform.PhaseBuilder
import org.beargineers.platform.PhasedAutonomous
import org.beargineers.platform.Phases
import org.beargineers.platform.Position
import org.beargineers.platform.action
import org.beargineers.platform.doOnce
import org.beargineers.platform.pathTo

@Autonomous(group = "Construct")
class ConstructAuto() : PhasedAutonomous<DecodeRobot>() {
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
                selectedActionsList.removeLast()
            }else if (selectedValueList.isNotEmpty()){
                selectedValueList.removeLast()
            }
        }

    }
    sealed class Option(val name: String) {
        class PhasesOption(name: String, val phases: Phases<DecodeRobot>) : Option(name)
        class FOption(name: String, val f: () -> Unit) : Option(name)
    }
    class Choice(val name: String, val list: List<Option.FOption>)
    lateinit var startingPoint: Position
    lateinit var launchPosition: Position
    lateinit var endPosition: Position
    val valueChoices = listOf(
        Choice("Alliance", listOf(
            Option.FOption("Red"){
                alliance = Alliance.RED
            },
            Option.FOption("Blue"){
                alliance = Alliance.BLUE
            }
        )),
        Choice(   "starting point:", listOf(
            Option.FOption("close"){
                startingPoint = AutoPositions.NORTH_START.mirrorForAlliance(robot)
            },
            Option.FOption("far"){
                startingPoint = AutoPositions.SOUTH_START.mirrorForAlliance(robot)
            }
        )),
        Choice(  "launch positions", listOf(
            Option.FOption("close"){
                launchPosition = AutoPositions.NORTH_SHOOTING.mirrorForAlliance(robot)
            },
            Option.FOption("far"){
                launchPosition = AutoPositions.SOUTH_SHOOTING.mirrorForAlliance(robot)
            }
        )),
        Choice(  "end positions", listOf(
            Option.FOption("open gate") {
                endPosition = Locations(robot).OPEN_RAMP_APPROACH.mirrorForAlliance(robot)
            },
            Option.FOption("box"){
                endPosition = AutoPositions.BOX_APPROACH.mirrorForAlliance(robot)
            }
        ))
    )
    val actionOptions = listOf(
        /*
                Choice("All next Shooting FAR"){
                    launchPosition = far.mirrorForAlliance(robot)
                },
                Choice("All next Shooting CLOSE"){
                    launchPosition = close.mirrorForAlliance(robot)
                },
        */
        Option.PhasesOption("Collect 1") {
            scoopAndShoot(1, launchPosition)
        },
        Option.PhasesOption("Collect 2") {
            scoopAndShoot(2, launchPosition)
        },
        Option.PhasesOption("Collect 3") {
            scoopAndShoot(3, launchPosition)
        },
        Option.PhasesOption("Collect From Box") {
            scoopFromBoxAndShoot(launchPosition)
        },
        Option.PhasesOption("Open Gate") {
            openRamp()
        }
    )
    val selectedValueList = mutableListOf<Option.FOption>()
    val selectedActionsList = mutableListOf<Option.PhasesOption>()
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
        // displays selections
        for ((i, choice) in valueChoices.withIndex()){
            telemetry.addData(choice.name, if (selectedValueList.size > i){selectedValueList[i].name}else{""})
        }
        for (i in 0..<selectedActionsList.size) {
            telemetry.addLine((i + 1).th() + ": " + selectedActionsList[i].name)
        }
        if (runningActionSelection) {   // displays action choices
            telemetry.addLine("Select:")
            for ((i, choice) in actionOptions.withIndex()) {
                if (i == (optionHighlight % actionOptions.size)) {
                    telemetry.addLine("  >" + choice.name)
                } else {
                    telemetry.addLine(choice.name)
                }
            }
        } else if (selectedValueList.size < valueChoices.size) {     // displays value choices
            telemetry.addLine("Please select the ${valueChoices[selectedValueList.size].name}:")
            for ((i, option) in valueChoices[selectedValueList.size].list.withIndex()) {
                if (i == (optionHighlight % valueChoices[selectedValueList.size].list.size)) {
                    telemetry.addLine("  >" + option.name)
                } else {
                    telemetry.addLine(option.name)
                }
            }
        } else {
            telemetry.addLine("Auto Program is saved")
        }
    }

    override fun PhaseBuilder<DecodeRobot>.phases() {
        for (choice in selectedValueList) {
            choice.f()
        }

        doOnce {
            enableFlywheel(true)
            intakeMode(IntakeMode.ON)
            assumePosition(startingPoint)
        }
        doWhile("Autonomous"){
            condition {
                opMode.elapsedTime.seconds() < 29
            }
            looping {
                followPathAndShoot(pathTo(launchPosition))
                for (choice in selectedActionsList) {
                    with(choice) {
                        phases()
                    }
                }
            }
            then{
                doOnce {
                    intakeMode(IntakeMode.OFF)
                    enableFlywheel(false)
                }
                action {
                    driveToTarget(endPosition)
                }
            }
        }
    }
}