package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.time.Duration

/**
 * # Phased Autonomous Framework
 *
 * A framework for creating structured, phase-based autonomous operations for FTC robots.
 * This system uses the **Composite design pattern** to allow you to break down complex
 * autonomous routines into discrete, sequential phases that can be organized hierarchically.
 *
 * ## Key Concepts
 *
 * ### What is a Phase?
 * A phase is a distinct step in your autonomous routine (e.g., "drive forward", "turn 90 degrees",
 * "deploy mechanism"). Each phase has:
 * - **Initialization**: Setup that runs once when the phase begins
 * - **Loop**: Logic that runs repeatedly until the phase completes
 *
 * ### The Composite Pattern
 * The framework uses the Composite pattern to support both:
 * - **Leaf phases**: Individual atomic operations (e.g., TurnPhase, GoPhase)
 * - **Composite phases**: Containers that hold and execute multiple child phases sequentially
 *
 * This allows you to:
 * - Nest phases hierarchically for better organization
 * - Reuse common phase sequences across different autonomous modes
 * - Create clear, readable autonomous code with named groups
 *
 * ### How Phases Work
 * 1. When a phase starts, `initPhase()` is called once to set up the phase
 * 2. Then `loopPhase()` is called repeatedly each loop cycle
 * 3. When `loopPhase()` returns `false`, the phase ends and the next phase begins
 * 4. Return `true` to keep the phase running
 * 5. For CompositePhase, it manages child phases automatically
 *
 * ## Creating a Leaf Phase
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
 * Use the Kotlin DSL builder to combine primitive phases into composites
 *
 * ```kotlin
 * @Autonomous
 * class DslAuto : PhasedAutonomous(phases("Autonomous") {
 *     composite("Navigate to scoring position") {
 *         phase(DriveForwardPhase(50.0, 0.6))
 *         phase(TurnPhase(90, 0.5))
 *         phase(DriveForwardPhase(30.0, 0.5))
 *     }
 *     phase(ScoreSamplePhase())
 *     composite("Return to observation zone") {
 *         phase(TurnPhase(180, 0.5))
 *         phase(DriveForwardPhase(80.0, 0.7))
 *     }
 * })
 * ```
 *
 * The framework automatically handles phase transitions and stops when all phases complete.
 */
interface AutonomousPhase {
    fun name(): String = javaClass.simpleName

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

class WaitPhase(private val durationInSeconds: Double) : AutonomousPhase {
    override fun Robot.initPhase() {
        drive.stop()
    }

    override fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean {
        return phaseTime.seconds() < durationInSeconds
    }
}

@PhaseDsl
fun PhaseBuilder.wait(duration: Duration) {
    phase(WaitPhase(duration.inWholeSeconds.toDouble()))
}

class SimpleActionPhase(private val action: Robot.() -> Unit) : AutonomousPhase {
    override fun Robot.initPhase() {
        action()
    }

    override fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean {
        return false
    }
}

@PhaseDsl
fun PhaseBuilder.action(action: Robot.() -> Unit) {
    phase(SimpleActionPhase(action))
}

@PhaseDsl
fun PhaseBuilder.assumePosition(position: Pose2D) {
    action {
        currentPose = position
    }
}

/**
 * A composite phase that executes a sequence of child phases.
 *
 * @param name Name for this composite phase (shown in telemetry)
 * @param phases List of child phases to execute in sequence
 */
class CompositePhase(
    private val _name: String,
    private val childPhases: List<AutonomousPhase>
) : AutonomousPhase {

    /** Index of the current child phase (0-based) */
    private var currentPhaseIdx = 0

    /** Whether the current child has been initialized */
    private var currentPhaseInitialized = false

    /** Timer for the current child phase */
    private val childPhaseTime = ElapsedTime()

    override fun name(): String = _name

    override fun Robot.initPhase() {
        // Reset state when the composite phase starts
        currentPhaseIdx = 0
        currentPhaseInitialized = false
    }

    override fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean {
        // If all child phases are complete, signal completion
        if (currentPhaseIdx >= childPhases.size) {
            return false
        }

        val currentPhase = childPhases[currentPhaseIdx]

        // Initialize the current child phase if needed
        if (!currentPhaseInitialized) {
            with(currentPhase) {
                telemetry.addLine("Current phase: ${currentPhase.name()}")
                initPhase()
            }
            currentPhaseInitialized = true
            childPhaseTime.reset()
        }

        // Execute the current child phase
        val continuePhase = with(currentPhase) {
            loopPhase(childPhaseTime)
        }

        // If child phase is done, move to next phase
        if (!continuePhase) {
            currentPhaseIdx++
            currentPhaseInitialized = false
        }

        telemetry.addData("Progress ${currentPhase.name()}", "${currentPhaseIdx + 1} / ${childPhases.size} %.2f s", phaseTime.seconds())

        // Continue composite phase as long as there are more child phases
        return true
    }
}

/**
 * DSL marker annotation for the phase builder DSL.
 *
 * This prevents accidental nesting of builder scopes and provides better
 * IDE support for the DSL.
 */
@DslMarker
annotation class PhaseDsl

/**
 * Builder class for constructing phases using a Kotlin DSL.
 */
@PhaseDsl
class PhaseBuilder {
    private val phases = mutableListOf<AutonomousPhase>()

    /**
     * Adds a single phase to the sequence.
     *
     * @param phase The phase to add
     */
    fun phase(phase: AutonomousPhase) {
        phases.add(phase)
    }

    /**
     * Creates a composite phase containing nested child phases.
     *
     * This function creates a new nested scope where you can define child phases.
     * All phases defined in the block become children of the composite.
     *
     * @param name The name for this composite phase (shown in telemetry)
     * @param block A lambda that builds the child phases
     */
    fun composite(name: String, block: PhaseBuilder.() -> Unit) {
        val builder = PhaseBuilder()
        builder.block()
        phases.add(CompositePhase(name, builder.build()))
    }

    /**
     * Builds and returns the list of phases.
     *
     * @return An immutable list of all phases added to this builder
     */
    internal fun build(): List<AutonomousPhase> = phases.toList()
}

/**
 * Top-level DSL function for building a phase sequence.
 *
 * This is the entry point for using the phase builder DSL. It creates a
 * PhaseBuilder, executes the provided block, and returns a CompositePhase
 * suitable for passing to PhasedAutonomous.
 *
 * ## Usage
 *
 * ```kotlin
 * @Autonomous(name = "My Auto")
 * class MyAuto : PhasedAutonomous(phases("Autonomous") {
 *     phase(GoPhase(30.0, 0.0, 0.5))
 *     phase(TurnPhase(90, 0.5))
 *     composite("Score") {
 *         phase(DeployArmPhase())
 *         phase(WaitPhase(1.0))
 *         phase(RetractArmPhase())
 *     }
 * })
 * ```
 *
 * @param name The name for the root composite phase
 * @param block A lambda that builds the phases using PhaseBuilder DSL
 * @return A CompositePhase instance containing all the built phases
 */
fun phases(name: String = "Autonomous", block: PhaseBuilder.() -> Unit): CompositePhase {
    val builder = PhaseBuilder()
    builder.block()
    return CompositePhase(name, builder.build())
}

abstract class PhasedAutonomous(alliance: Alliance, private val rootPhase: CompositePhase) : Robot(alliance) {

    /** Whether the root phase has been initialized */
    private var initialized = false

    /** Timer tracking elapsed time for the entire autonomous */
    private val totalTime = ElapsedTime()

    override fun loop() {
        super.loop()

        // Initialize the root phase on first loop
        if (!initialized) {
            with(rootPhase) {
                initPhase()
            }
            totalTime.reset()
            initialized = true
            telemetry.addData("Phase", rootPhase.name())
        }

        // Execute the root phase
        val continueRunning = with(rootPhase) {
            loopPhase(totalTime)
        }

        // Stop when complete
        if (!continueRunning) {
            telemetry.addData("Status", "Complete!")
            stop()
        }
    }
}