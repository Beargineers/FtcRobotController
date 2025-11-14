package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.util.ElapsedTime

/**
 * # Phased Autonomous Framework
 *
 * A framework for creating structured, phase-based autonomous operations for FTC robots.
 * This system allows you to break down complex autonomous routines into discrete, sequential phases
 * that execute one after another.
 *
 * ## Key Concepts
 *
 * ### What is a Phase?
 * A phase is a distinct step in your autonomous routine (e.g., "drive forward", "turn 90 degrees",
 * "deploy mechanism"). Each phase has:
 * - **Initialization**: Setup that runs once when the phase begins
 * - **Loop**: Logic that runs repeatedly until the phase completes
 *
 * ### How Phases Work
 * 1. When a phase starts, `initPhase()` is called once to set up the phase
 * 2. Then `loopPhase()` is called repeatedly each loop cycle
 * 3. When `loopPhase()` returns `false`, the phase ends and the next phase begins
 * 4. Return `true` to keep the phase running
 *
 * ## Creating a Phase
 *
 * Implement the `AutonomousPhase` interface:
 *
 * ```kotlin
 * class DriveForwardPhase(val distanceCm: Double, val power: Double) : AutonomousPhase {
 *     override fun Robot.initPhase() {
 *         // Called once when phase starts
 *         drive.goto(distanceCm, 0.0, power)
 *     }
 *
 *     override fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean {
 *         // Called repeatedly - return true to continue, false when done
 *         return drive.allMotors.any { it.isBusy }
 *     }
 * }
 * ```
 *
 * ## Using the Framework
 *
 * Create an autonomous OpMode by extending `PhasedAutonomous` and passing your phases:
 *
 * ```kotlin
 * @Autonomous
 * class MyAutonomous : PhasedAutonomous(
 *     DriveForwardPhase(30.0, 0.5),
 *     TurnPhase(90, 0.5),
 *     DriveForwardPhase(20.0, 0.5)
 * )
 * ```
 *
 * The framework automatically handles phase transitions and stops when all phases complete.
 */
interface AutonomousPhase {
    /**
     * Called once when this phase begins.
     *
     * Use this to initialize the phase - set motor targets, reset sensors,
     * start timers, or any other one-time setup.
     *
     * This method is a receiver function on Robot, so you have direct access
     * to all Robot properties (drive, hardware, telemetry, etc.).
     */
    fun Robot.initPhase()

    /**
     * Called repeatedly while this phase is active.
     *
     * @param phaseTime Elapsed time since this phase started
     * @return `true` to continue the phase, `false` to end it and move to the next phase
     *
     * Use this to monitor progress and determine when the phase should end.
     * Common patterns:
     * - Check if motors are busy: `return drive.allMotors.any { it.isBusy }`
     * - Use a timeout: `return phaseTime.seconds() < 5.0`
     * - Check sensor values: `return colorSensor.alpha() < threshold`
     *
     * This method is a receiver function on Robot, so you have direct access
     * to all Robot properties.
     */
    fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean
}

/**
 * Base class for phase-based autonomous OpModes.
 *
 * This class manages the execution of a sequence of [AutonomousPhase]s, automatically
 * transitioning between them and handling the phase lifecycle.
 *
 * ## How It Works
 *
 * 1. Pass phases to the constructor in the order you want them to execute
 * 2. The framework automatically calls `initPhase()` when entering each phase
 * 3. Calls `loopPhase()` repeatedly until it returns false
 * 4. Moves to the next phase and repeats
 * 5. Stops automatically when all phases complete
 *
 * ## Telemetry
 *
 * The framework automatically provides telemetry showing:
 * - Current phase name
 * - Time elapsed in current phase
 * - Total time elapsed
 *
 * ## Example
 *
 * ```kotlin
 * @Autonomous(name = "Red Left")
 * class RedLeftAuto : PhasedAutonomous(
 *     DriveForwardPhase(30.0, 0.5),  // Drive forward 30cm
 *     TurnPhase(90, 0.5),              // Turn 90 degrees
 *     DeployArmPhase(),                // Custom mechanism phase
 *     WaitPhase(2.0)                   // Wait 2 seconds
 * )
 * ```
 *
 * @param phases Vararg of phases to execute in sequence
 */
abstract class PhasedAutonomous(vararg phases: AutonomousPhase) : Robot() {
    /** List of all phases to execute */
    val phases: List<AutonomousPhase> = phases.toList()

    /** Index of the current phase (0-based) */
    var currentPhaseIdx = 0

    /** The currently executing phase, or null if transitioning */
    var currentPhase: AutonomousPhase? = null

    /** Timer tracking elapsed time in the current phase */
    val phaseTime = ElapsedTime()

    override fun loop() {
        super.loop()

        if (currentPhaseIdx >= phases.size) return

        if (currentPhase == null) {
            currentPhase = phases[currentPhaseIdx]
            with(currentPhase!!) {
                initPhase()
            }
            phaseTime.reset()
            telemetry.addData("Phase: ", currentPhase!!.javaClass.simpleName)
        }

        with(currentPhase!!) {
            if (!loopPhase(phaseTime)) {
                currentPhaseIdx++
                currentPhase = null

                if (currentPhaseIdx >= phases.size) {
                    stop()
                }
            }
        }

        telemetry.addData("Elapsed phase", "$elapsed")
        telemetry.addData("Elapsed total", "$phaseTime")
    }
}