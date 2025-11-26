package org.beargineers.platform

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.cos
import kotlin.math.sin
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
 * A phase is a distinct step in your autonomous routine (e.g., "drive to position", "wait",
 * "deploy mechanism"). Each phase has:
 * - **Initialization**: Setup that runs once when the phase begins
 * - **Loop**: Logic that runs repeatedly until the phase completes
 *
 * ### The Composite Pattern
 * The framework uses the Composite pattern to support both:
 * - **Leaf phases**: Individual atomic operations (e.g., GotoPosePhase, WaitPhase)
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
 * ## Built-in Phases
 *
 * The framework provides several ready-to-use phases:
 * - **WaitPhase**: Pauses execution for a specified duration
 * - **SimpleActionPhase**: Executes a single action immediately
 * - **GotoPosePhase**: Drives the robot to a target position on the field
 * - **DriveRelative**: Moves the robot relative to its current position
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
 * - **`composite(name) { ... }`**: Group phases together with a name
 *
 * The framework automatically handles phase transitions and stops when all phases complete.
 */
interface AutonomousPhase<in Robot: BaseRobot> {
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
    fun Robot.initPhase()

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
    fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean
}

class WaitPhase(private val durationInSeconds: Double) : AutonomousPhase<BaseRobot> {
    override fun BaseRobot.initPhase() {
        drive.stop()
    }

    override fun BaseRobot.loopPhase(phaseTime: ElapsedTime): Boolean {
        return phaseTime.seconds() < durationInSeconds
    }
}

@PhaseDsl
fun PhaseBuilder<*>.wait(duration: Duration) {
    phase(WaitPhase(duration.inWholeSeconds.toDouble()))
}

class SimpleActionPhase<Robot: BaseRobot>(private val action: Robot.() -> Unit) : AutonomousPhase<Robot> {
    override fun Robot.initPhase() {
        action()
    }

    override fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean {
        return false
    }
}

@PhaseDsl
fun <Robot: BaseRobot> PhaseBuilder<Robot>.action(action: Robot.() -> Unit) {
    phase(SimpleActionPhase(action))
}

@PhaseDsl
fun <Robot: BaseRobot> PhaseBuilder<Robot>.assumePosition(position: Position) {
    action {
        currentPosition = position
    }
}

class GotoPosePhase(val pose: Position, val maxSpeed: Double) : AutonomousPhase<BaseRobot> {
    override fun BaseRobot.initPhase() {
    }

    override fun BaseRobot.loopPhase(phaseTime: ElapsedTime): Boolean {
        return driveToTarget(pose, maxSpeed)
    }
}

@PhaseDsl
fun PhaseBuilder<*>.driveTo(pose: Position, maxSpeed: Double = 1.0) {
    phase(GotoPosePhase(pose, maxSpeed))
}

class DriveRelative<R: BaseRobot>(val movement: RobotMovement,
                                  val maxSpeed: Double) :
    AutonomousPhase<R> {
    lateinit var targetPosition: Position

    override fun R.initPhase() {
        val cp = currentPosition.toDistanceUnit(movement.distanceUnit).toAngleUnit(AngleUnit.RADIANS)
        val t = if (movement.angleUnit == AngleUnit.RADIANS) movement.turn else Math.toRadians(movement.turn)
        targetPosition = Position(
            // Forward: all wheels contribute equally
            x = cp.x + movement.forward * cos(cp.heading) + movement.right * sin(cp.heading),
            // Strafe: diagonal wheels oppose (LF and RB forward = strafe right)
            y = cp.y + movement.forward * sin(cp.heading) - movement.right * cos(cp.heading),
            // Get yaw in radians to match the angleUnit specification
            heading = cp.heading + t,
            distanceUnit = movement.distanceUnit,
            angleUnit = AngleUnit.RADIANS
        )
    }

    override fun R.loopPhase(phaseTime: ElapsedTime): Boolean {
        return driveToTarget(targetPosition, maxSpeed)
    }
}

@PhaseDsl
fun <R: BaseRobot> PhaseBuilder<R>.driveRelative(movement: RobotMovement, maxSpeed: Double = 0.5) {
    phase(DriveRelative(movement, maxSpeed))
}
/**
 * A composite phase that executes a sequence of child phases.
 *
 * @param name Name for this composite phase (shown in telemetry)
 * @param phases List of child phases to execute in sequence
 */
class CompositePhase<Robot: BaseRobot>(
    private val _name: String,
    private val childPhases: List<AutonomousPhase<Robot>>
) : AutonomousPhase<Robot> {

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

typealias Phases<Robot> = PhaseBuilder<Robot>.() -> Unit

/**
 * Builder class for constructing phases using a Kotlin DSL.
 */
@PhaseDsl
class PhaseBuilder<Robot: BaseRobot> {
    private val phases = mutableListOf<AutonomousPhase<Robot>>()

    /**
     * Adds a single phase to the sequence.
     *
     * @param phase The phase to add
     */
    fun phase(phase: AutonomousPhase<Robot>) {
        phases.add(phase)
    }

    /**
     * Creates a composite phase containing nested child phases.
     *
     * This function creates a new nested scope where you can define child phases.
     * All phases defined in the block become children of the composite.
     *
     * @param name The name for this composite phase (shown in telemetry)
     * @param phases A lambda that builds the child phases
     */
    fun composite(name: String, phases: Phases<Robot>) {
        val builder = PhaseBuilder<Robot>()
        builder.phases()
        this@PhaseBuilder.phases.add(CompositePhase(name, builder.build()))
    }

    /**
     * Builds and returns the list of phases.
     *
     * @return An immutable list of all phases added to this builder
     */
    internal fun build(): List<AutonomousPhase<Robot>> = phases.toList()
}

abstract class PhasedAutonomous<Robot: BaseRobot>(alliance: Alliance, phases: Phases<Robot>) : RobotOpMode<Robot>(alliance) {
    val rootPhase by lazy {
        val builder = PhaseBuilder<Robot>()
        with(builder) {
            phases()
        }
        CompositePhase("Plan", builder.build())
    }

    /** Whether the root phase has been initialized */
    private var initialized = false

    /** Timer tracking elapsed time for the entire autonomous */
    private val totalTime = ElapsedTime()

    override fun loop() {
        super.loop()

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