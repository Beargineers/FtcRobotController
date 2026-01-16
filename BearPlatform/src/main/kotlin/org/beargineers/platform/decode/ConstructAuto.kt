package org.beargineers.platform.decode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.beargineers.platform.Alliance
import org.beargineers.platform.PhaseBuilder
import org.beargineers.platform.PhasedAutonomous
import org.beargineers.platform.Phases
import org.beargineers.platform.doOnce
import org.beargineers.platform.tilePosition

open class ConstructAuto(alliance: Alliance) : PhasedAutonomous<DecodeRobot>(alliance) {
    override fun bearInit() {
        button(gamepad1::a) {
            if (runningSelection) {
                selectedList.add(choices[optionHighlight])
            }
        }
        button(gamepad1::dpad_right) {
            if (optionHighlight < choices.size - 1) {
                optionHighlight += 1
            }
        }
        button(gamepad1::dpad_left) {
            if (optionHighlight > 0) {
                optionHighlight -= 1
            }
        }
        button(gamepad1::x) {
            if (runningSelection) {
                runningSelection = false
            }
        }

    }
    class Choice(val name: String, val phases: Phases<DecodeRobot>)

    val far = tilePosition("D1:+160")
    val close = tilePosition("D4:+135")
    var launchPosition = far.mirrorForAlliance(robot)

    val choices = listOf(
        Choice("All next Shooting FAR"){
            launchPosition = far.mirrorForAlliance(robot)
        },
        Choice("All next Shooting CLOSE"){
            launchPosition = close.mirrorForAlliance(robot)
        },
        Choice("Collect 1") {
            scoopAndShoot(1, launchPosition)
        },
        Choice("Collect 2") {
            scoopAndShoot(2, launchPosition)
        },
        Choice("Collect 3") {
            scoopAndShoot(3, launchPosition)
        },
        Choice("Collect From Box") {
            scoopFromBoxAndShoot(launchPosition)
        },
        Choice("Open Gate") {
            openRamp()
        }
        )
    val selectedList = mutableListOf<Choice>()
    var optionHighlight = 1

    var runningSelection = true

    override fun init_loop() {
        if (runningSelection) {
            super.init_loop()
            var selected = ""
            for (i in 0..<selectedList.size) {
                selected += selectedList[i].name + ", "
            }

            telemetry.addData("Selections: ", selected)

            allButtons.forEach { it.update() }

            var selectionString = ""
            for ((i, choice) in choices.withIndex()) {
                if (i == optionHighlight) {
                    selectionString += " __" + choice.name.uppercase() + "__ "
                } else {
                    selectionString += choice.name + ", "
                }
            }
            telemetry.addData("Select the new step: ", selectionString)
        } else {
            telemetry.addData("Auto Program", " Saved")
        }
    }

    override fun PhaseBuilder<DecodeRobot>.phases() {
        doOnce {
            enableFlywheel(true)
            intakeMode(IntakeMode.ON)
        }

        for (choice in selectedList) {
            with(choice) {
                phases()
            }
        }
    }
}

@Autonomous
class ConstructBlue : ConstructAuto(Alliance.BLUE)

@Autonomous
class ConstructRed : ConstructAuto(Alliance.RED)