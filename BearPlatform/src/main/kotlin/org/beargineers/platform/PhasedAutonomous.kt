package org.beargineers.platform

import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.time.Duration

/**
 * # Phased Autonomous Framework
 *
 * A framework for creating structured, phase-based autonomous operations for FTC robots.
 * This system uses the **Composite design pattern** to allow you to break down complex
 * autonomous routines into discrete phases that can be organized hierarchically and executed
 * sequentially or in parallel.
 *
 * ## Key Concepts
 *
 * ### What is a Phase?
 * A phase is a distinct step in your autonomous routine (e.g., "drive to position", "wait",
 * "deploy mechanism"). Each phase has:
 * - **Initialization**: Setup that runs once when the phase begins
 * - **Loop**: Logic that runs repeatedly until the phase completes
 *
 * ### The Composite Pattern
 * The framework uses the Composite pattern to support:
 * - **Leaf phases**: Individual atomic operations (e.g., GotoPosePhase, WaitPhase)
 * - **SequentialPhase**: Containers that execute multiple child phases one after another
 * - **ParallelPhase**: Containers that execute multiple child phases concurrently
 *
 * This allows you to:
 * - Nest phases hierarchically for better organization
 * - Run multiple operations simultaneously (e.g., deploy arm while driving)
 * - Reuse common phase sequences across different autonomous modes
 * - Create clear, readable autonomous code with named groups
 *
 * ### How Phases Work
 * 1. When a phase starts, `initPhase()` is called once to set up the phase
 * 2. Then `loopPhase()` is called repeatedly each loop cycle
 * 3. When `loopPhase()` returns `false`, the phase ends and the next phase begins
 * 4. Return `true` to keep the phase running
 * 5. For SequentialPhase, child phases run one at a time in order
 * 6. For ParallelPhase, all child phases run together until all complete
 *
 * ## Built-in Phases
 *
 * The framework provides several ready-to-use phases:
 * - **WaitPhase**: Pauses execution for a specified duration
 * - **SimpleActionPhase**: Executes a single action immediately
 * - **GotoPosePhase**: Drives the robot to a target position on the field
 * - **DriveRelative**: Moves the robot relative to its current position
 * - **SequentialPhase**: Executes child phases one after another
 * - **ParallelPhase**: Executes child phases concurrently
 * - **DoWhilePhase**: Loops while condition is true, then executes another phase
 *
 * ## Creating Custom Phases
 *
 * Implement the `AutonomousPhase<Robot>` interface with your robot type:
 *
 * ```kotlin
 * class DeployArmPhase : AutonomousPhase<MyRobot> {
 *     override fun MyRobot.initPhase() {
 *         // Called once when phase starts
 *         armMotor.targetPosition = ARM_DEPLOY_POSITION
 *         armMotor.power = 0.5
 *     }
 *
 *     override fun MyRobot.loopPhase(phaseTime: ElapsedTime): Boolean {
 *         // Called repeatedly - return true to continue, false when done
 *         return armMotor.isBusy
 *     }
 * }
 * ```
 *
 * ## Using the DSL (Recommended)
 *
 * Create autonomous routines using the Kotlin DSL by overriding `createPhases()`:
 *
 * ```kotlin
 * @Autonomous(name = "My Auto")
 * class MyAutonomous : PhasedAutonomous<MyRobot>(Alliance.RED) {
 *     override fun PhaseBuilder<MyRobot>.createPhases() {
 *         // Drive to spike marks
 *         driveTo(SPIKE_POSITION, maxSpeed = 0.5)
 *
 *         // Wait and pick up sample
 *         wait(1.seconds)
 *         action { intake.setPower(1.0) }
 *
 *         // Organize complex sequences
 *         composite("Score specimen") {
 *             driveTo(CHAMBER_POSITION)
 *             action { arm.deploy() }
 *             wait(0.5.seconds)
 *             action { claw.release() }
 *         }
 *
 *         // Run multiple actions concurrently
 *         parallel("Deploy and drive") {
 *             action { arm.lower() }
 *             action { intake.start() }
 *             driveTo(PICKUP_POSITION)
 *         }
 *
 *         // Park
 *         driveTo(OBSERVATION_ZONE)
 *     }
 * }
 * ```
 *
 * ## DSL Functions
 *
 * - **`driveTo(pose, maxSpeed)`**: Drive to an absolute field position
 * - **`driveRelative(forward, right, turn, ...)`**: Drive relative to current position
 * - **`wait(duration)`**: Wait for a specified duration
 * - **`action { ... }`**: Execute a single immediate action
 * - **`assumePosition(position)`**: Set the robot's believed position (e.g., at start)
 * - **`seq(name) { ... }`**: Group phases to execute sequentially
 * - **`par(name) { ... }`**: Group phases to execute concurrently
 * - **`doWhile(name) { condition { ... } looping { ... } then { ... } }`**: Loop while condition, then continue
 *
 * The framework automatically handles phase transitions and stops when all phases complete.
 */
interface AutonomousPhase<in R: Robot> {
    /**
     * Returns the name of this phase for display in telemetry.
     *
     * By default, returns the simple class name. Override to provide a custom name.
     */
    fun name(): String = javaClass.simpleName

    /**
     * Called once when this phase begins.
     *
     * Use this to initialize the phase - set motor targets, reset sensors,
     * start timers, calculate trajectories, or any other one-time setup.
     *
     * This method is a receiver function on Robot, so you have direct access
     * to all Robot properties (drive, currentPosition, telemetry, etc.).
     *
     * @receiver Robot The robot instance this phase will control
     */
    fun R.initPhase()

    /**
     * Called repeatedly while this phase is active (every loop iteration).
     *
     * This method should check the phase's completion condition and return:
     * - `true` to continue running the phase
     * - `false` to signal completion and transition to the next phase
     *
     * @param phaseTime Elapsed time since this phase started (resets at 0 for each phase)
     * @return `true` to continue the phase, `false` to end it and move to the next phase
     *
     * @receiver Robot The robot instance this phase is controlling
     *
     * ## Common Patterns
     *
     * ### Wait for duration
     * ```kotlin
     * return phaseTime.seconds() < 5.0
     * ```
     *
     * ### Wait for target reached
     * ```kotlin
     * return !driveToTarget(targetPosition, maxSpeed)
     * ```
     *
     * ### Immediate completion (action phases)
     * ```kotlin
     * return false  // Complete immediately after initPhase()
     * ```
     *
     * ### Wait for condition
     * ```kotlin
     * return !colorSensor.isDetected()
     * ```
     */
    fun R.loopPhase(phaseTime: ElapsedTime): Boolean
}

class WaitPhase(private val durationInMilliseconds: Long) : AutonomousPhase<Robot> {
    override fun Robot.initPhase() {
    }

    override fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean {
        return phaseTime.milliseconds() < durationInMilliseconds
    }
}

@PhaseDsl
fun PhaseBuilder<*>.wait(duration: Duration) {
    phase(WaitPhase(duration.inWholeMilliseconds))
}

class SimpleActionPhase<R: Robot>(private val action: R.() -> Boolean) : AutonomousPhase<R> {
    override fun R.initPhase() {
    }

    override fun R.loopPhase(phaseTime: ElapsedTime): Boolean {
        return action()
    }
}

@PhaseDsl
fun <R: Robot> PhaseBuilder<R>.action(action: R.() -> Boolean) {
    phase(SimpleActionPhase(action))
}

@PhaseDsl
fun <R: Robot> PhaseBuilder<R>.doOnce(action: R.() -> Unit) {
    phase(SimpleActionPhase({action(); false}))
}

@PhaseDsl
fun <R: Robot> PhaseBuilder<R>.assumeRobotPosition(position: Position) {
    doOnce {
        assumePosition(position)
    }
}

class GotoPositionViaPhase(val waypoints: List<Waypoint>): AutonomousPhase<Robot> {
    override fun Robot.initPhase() {
    }

    override fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean {
        return followPath(waypoints).also { targetSpeed = 1.0 }
    }
}

@PhaseDsl
fun PhaseBuilder<*>.drive(waypoints: List<Waypoint>) {
    phase(GotoPositionViaPhase(waypoints))
}

class DriveRelative<R: Robot>(val movement: RelativePosition) : AutonomousPhase<R> {
    lateinit var targetPosition: Position

    override fun R.initPhase() {
        targetPosition = currentPosition + movement
    }

    override fun R.loopPhase(phaseTime: ElapsedTime): Boolean {
        return driveToTarget(targetPosition)
    }
}

@PhaseDsl
fun <R: Robot> PhaseBuilder<R>.driveRelative(movement: RelativePosition) {
    phase(DriveRelative(movement))
}

/**
 * A composite phase that executes a sequence of child phases sequentially.
 *
 * @param name Name for this composite phase (shown in telemetry)
 * @param childPhases List of child phases to execute in sequence
 */
class SequentialPhase<R: Robot>(
    private val _name: String,
    private val childPhases: List<AutonomousPhase<R>>
) : AutonomousPhase<R> {

    /** Index of the current child phase (0-based) */
    private var currentPhaseIdx = 0

    /** Whether the current child has been initialized */
    private var currentPhaseInitialized = false

    /** Timer for the current child phase */
    private val childPhaseTime = ElapsedTime()

    override fun name(): String = _name

    override fun R.initPhase() {
        // Reset state when the composite phase starts
        currentPhaseIdx = 0
        currentPhaseInitialized = false
    }

    override fun R.loopPhase(phaseTime: ElapsedTime): Boolean {
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
 * A phase that runs one set of phases while a condition is true, then switches
 * permanently to another set of phases once the condition becomes false.
 *
 * The condition is evaluated on every loop iteration while running the "while" phase.
 * Once the condition becomes false, execution switches to the "then" phase permanently
 * (no switching back). This is useful for behaviors like "keep searching while not found,
 * then score".
 *
 * @param name Name for this phase (shown in telemetry)
 * @param condition Function that returns true to continue the "while" phase
 * @param whilePhase Phase to execute while condition is true
 * @param thenPhase Phase to execute after condition becomes false
 */
class DoWhilePhase<R: Robot>(
    private val _name: String,
    private val condition: R.() -> Boolean,
    private val whilePhase: AutonomousPhase<R>,
    private val thenPhase: AutonomousPhase<R>
) : AutonomousPhase<R> {

    /** Whether we've switched to the "then" phase */
    private var switchedToThen = false

    /** Whether the current phase has been initialized */
    private var currentPhaseInitialized = false

    /** Timer for the current child phase */
    private val childPhaseTime = ElapsedTime()

    override fun name(): String = _name

    override fun R.initPhase() {
        switchedToThen = false
        currentPhaseInitialized = false
    }

    override fun R.loopPhase(phaseTime: ElapsedTime): Boolean {
        // Check condition only if we haven't switched yet
        if (!switchedToThen && !condition()) {
            switchedToThen = true
            currentPhaseInitialized = false
            telemetry.addLine("'$_name': condition false -> ${thenPhase.name()}")
        }

        val currentPhase = if (switchedToThen) thenPhase else whilePhase

        // Initialize current phase if needed
        if (!currentPhaseInitialized) {
            with(currentPhase) {
                initPhase()
            }
            childPhaseTime.reset()
            currentPhaseInitialized = true
        }

        // Delegate to the current phase
        val continuePhase = with(currentPhase) {
            loopPhase(childPhaseTime)
        }

        // If whilePhase completes, switch to thenPhase instead of ending
        if (!continuePhase && !switchedToThen) {
            switchedToThen = true
            currentPhaseInitialized = false
            telemetry.addLine("'$_name': while phase complete -> ${thenPhase.name()}")
            return true  // Continue with thenPhase
        }

        // Return the result from the current phase
        return continuePhase
    }
}

/**
 * Builder for constructing a "doWhile" phase using a DSL.
 *
 * Example usage:
 * ```kotlin
 * doWhile("Search for sample") {
 *     condition { !sampleDetected() }
 *     looping {
 *         // Keep searching while condition is true
 *         driveTo(SEARCH_POSITION)
 *     }
 *     then {
 *         // Execute after condition becomes false
 *         action { grabSample() }
 *         driveTo(SCORING_POSITION)
 *     }
 * }
 * ```
 */
@PhaseDsl
class DoWhileBuilder<R: Robot>(val opMode: RobotOpMode<R>) {
    private var conditionBlock: (R.() -> Boolean)? = null
    private var whilePhases: Phases<R>? = null
    private var thenPhases: Phases<R>? = null

    /**
     * Sets the condition to evaluate. The "looping" phase runs while this returns true.
     *
     * @param block A lambda that returns true to continue looping
     */
    fun condition(block: R.() -> Boolean) {
        conditionBlock = block
    }

    /**
     * Sets the phases to execute while the condition is true.
     *
     * @param phases A lambda that builds the "while" phases
     */
    fun looping(phases: Phases<R>) {
        whilePhases = phases
    }

    /**
     * Sets the phases to execute after the condition becomes false.
     *
     * @param phases A lambda that builds the "then" phases
     */
    fun then(phases: Phases<R>) {
        thenPhases = phases
    }

    /**
     * Builds the doWhile phase.
     *
     * @param name The name for this phase
     * @return The constructed DoWhilePhase
     * @throws IllegalStateException if condition, looping, or then is not set
     */
    internal fun build(name: String): DoWhilePhase<R> {
        val condition = conditionBlock
            ?: throw IllegalStateException("doWhile '$name' requires a condition block")
        val whileBlock = whilePhases
            ?: throw IllegalStateException("doWhile '$name' requires a looping block")
        val thenBlock = thenPhases
            ?: throw IllegalStateException("doWhile '$name' requires a then block")

        val whileBuilder = PhaseBuilder<R>(opMode)
        whileBuilder.whileBlock()
        val whilePhase = SequentialPhase("$name:while", whileBuilder.build())

        val thenBuilder = PhaseBuilder<R>(opMode)
        thenBuilder.thenBlock()
        val thenPhase = SequentialPhase("$name:then", thenBuilder.build())

        return DoWhilePhase(name, condition, whilePhase, thenPhase)
    }
}

/**
 * A parallel phase that executes multiple child phases concurrently.
 *
 * All child phases start together and run in parallel until ALL phases complete.
 * All phases share the same timer since they start simultaneously.
 *
 * @param name Name for this parallel phase (shown in telemetry)
 * @param childPhases List of child phases to execute in parallel
 */
class ParallelPhase<R: Robot>(
    private val _name: String,
    private val childPhases: List<AutonomousPhase<R>>
) : AutonomousPhase<R> {

    /** Track which phases have completed */
    private val completed = BooleanArray(childPhases.size) { false }
    private val childPhaseTime = ElapsedTime()

    override fun name(): String = _name

    override fun R.initPhase() {
        // Initialize all child phases at once
        for (i in childPhases.indices) {
            completed[i] = false
            with(childPhases[i]) {
                initPhase()
            }
        }
        childPhaseTime.reset()
    }

    override fun R.loopPhase(phaseTime: ElapsedTime): Boolean {
        // Execute all child phases in parallel
        for (i in childPhases.indices) {
            // Skip phases that have already completed
            if (completed[i]) continue

            val phase = childPhases[i]

            // Execute phase with shared timer
            val continuePhase = with(phase) {
                loopPhase(childPhaseTime)
            }

            if (!continuePhase) {
                completed[i] = true
            }
        }

        // Show progress
        val completedCount = completed.count { it }
        telemetry.addData("Progress $_name", "$completedCount / ${childPhases.size} complete")

        // Continue until all phases complete
        return completedCount < childPhases.size
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

typealias Phases<Robot> = PhaseBuilder<Robot>.() -> Unit

/**
 * Builder class for constructing phases using a Kotlin DSL.
 */
@PhaseDsl
class PhaseBuilder<R: Robot>(val opMode: RobotOpMode<R>) {
    private val phases = mutableListOf<AutonomousPhase<R>>()

    val robot: R get() = opMode.robot

    /**
     * Adds a single phase to the sequence.
     *
     * @param phase The phase to add
     */
    fun phase(phase: AutonomousPhase<R>) {
        phases.add(phase)
    }

    /**
     * Creates a composite phase containing nested child phases that run sequentially.
     *
     * This function creates a new nested scope where you can define child phases.
     * All phases defined in the block become children of the composite and execute
     * one after another.
     *
     * @param name The name for this composite phase (shown in telemetry)
     * @param phases A lambda that builds the child phases
     */
    fun seq(name: String, phases: Phases<R>) {
        val builder = PhaseBuilder<R>(opMode)
        builder.phases()
        this@PhaseBuilder.phases.add(SequentialPhase(name, builder.build()))
    }

    /**
     * Creates a parallel phase containing nested child phases that run concurrently.
     *
     * This function creates a new nested scope where you can define child phases.
     * All phases defined in the block start together and run simultaneously until
     * all phases complete.
     *
     * @param name The name for this parallel phase (shown in telemetry)
     * @param phases A lambda that builds the child phases
     */
    fun par(name: String, phases: Phases<R>) {
        val builder = PhaseBuilder<R>(opMode)
        builder.phases()
        this@PhaseBuilder.phases.add(ParallelPhase(name, builder.build()))
    }

    /**
     * Creates a phase that loops while a condition is true, then executes another phase.
     *
     * The condition is evaluated on every loop. While true, the `looping` phase runs.
     * Once the condition becomes false, execution switches permanently to the `then` phase.
     *
     * Example usage:
     * ```kotlin
     * doWhile("Search for sample") {
     *     condition { !sampleDetected() }
     *     looping {
     *         driveTo(SEARCH_POSITION)
     *     }
     *     then {
     *         action { grabSample() }
     *         driveTo(SCORING_POSITION)
     *     }
     * }
     * ```
     *
     * @param name The name for this phase (shown in telemetry)
     * @param block A lambda that configures the condition and phases
     */
    fun doWhile(name: String, block: DoWhileBuilder<R>.() -> Unit) {
        val builder = DoWhileBuilder<R>(opMode)
        builder.block()
        this@PhaseBuilder.phases.add(builder.build(name))
    }

    /**
     * Builds and returns the list of phases.
     *
     * @return An immutable list of all phases added to this builder
     */
    internal fun build(): List<AutonomousPhase<R>> = phases.toList()
}

abstract class PhasedAutonomous<R: Robot>(alliance: Alliance) : RobotOpMode<R>(alliance) {
    val rootPhase by lazy {
        val builder = PhaseBuilder<R>(this)
        with(builder) {
            phases()
        }
        SequentialPhase("Plan", builder.build())
    }

    abstract fun PhaseBuilder<R>.phases()

    /** Whether the root phase has been initialized */
    private var initialized = false

    /** Timer tracking elapsed time for the entire autonomous */
    private val totalTime = ElapsedTime()

    override fun bearLoop() {
        super.bearLoop()

        // Initialize the root phase on first loop
        if (!initialized) {
            with(rootPhase) {
                robot.initPhase()
            }
            totalTime.reset()
            initialized = true
            telemetry.addData("Phase", rootPhase.name())
        }

        // Execute the root phase
        val continueRunning = with(rootPhase) {
            robot.loopPhase(totalTime)
        }

        // Stop when complete
        if (!continueRunning) {
            telemetry.addData("Status", "Complete!")
            stop()
        }
    }
}