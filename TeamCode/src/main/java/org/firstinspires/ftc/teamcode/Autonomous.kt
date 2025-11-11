package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime

/**
 * # Example Autonomous Phases and OpMode
 *
 * This file demonstrates how to use the PhasedAutonomous framework with example phase
 * implementations. These examples show common autonomous operations for FTC robots.
 *
 * ## Understanding the Examples
 *
 * ### TurnPhase
 * A reusable phase that turns the robot a specified number of degrees.
 * - Supports both positive (counter-clockwise) and negative (clockwise) rotations
 * - Uses encoder-based turning for accuracy
 * - Completes when motors finish turning
 *
 * ### GoPhase
 * A reusable phase that moves the robot to a position relative to its current location.
 * - Uses mecanum drive kinematics to move in any direction
 * - Parameters are in centimeters (ycm = forward/back, xcm = left/right)
 * - Uses encoders for position control
 *
 * ### Example Autonomous
 * A complete autonomous OpMode that combines phases to create a simple routine:
 * 1. Drive forward 12cm
 * 2. Turn 180 degrees
 * 3. Drive forward 12cm (now going back to start)
 *
 * ## Creating Your Own Phases
 *
 * Follow this pattern:
 * ```kotlin
 * class MyPhase(val param1: Type, val param2: Type) : AutonomousPhase {
 *     override fun Robot.initPhase() {
 *         // One-time setup: set motor targets, initialize sensors, etc.
 *     }
 *
 *     override fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean {
 *         // Return true to continue, false when done
 *     }
 * }
 * ```
 *
 * ## Tips for Phase Design
 *
 * - **Keep phases focused**: Each phase should do one thing well
 * - **Make them reusable**: Use parameters so phases can be used in different situations
 * - **Use appropriate completion conditions**:
 *   - Motor busy state for movement phases
 *   - Timers for wait phases
 *   - Sensor readings for detection phases
 * - **Add telemetry**: Use telemetry.addData() in loopPhase() for debugging
 */

/**
 * Phase that turns the robot by a specified angle.
 *
 * This phase uses the drivetrain's encoder-based turn function to rotate
 * the robot. The phase completes when all motors finish their rotation.
 *
 * @param degrees Angle to turn in degrees. Positive = counter-clockwise, negative = clockwise
 * @param power Motor power (0.0 to 1.0) to use for turning
 *
 * @see Drivebase.turn
 */
class TurnPhase(val degrees: Int, val power: Double) : AutonomousPhase {
    override fun Robot.initPhase() {
        // Initialize the turn using the drivetrain's turn function
        // This sets motor targets for the rotation
        drive.turn(degrees, power)
    }

    override fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean {
        // Continue the phase while any motor is still busy turning
        // Returns true to keep running, false when all motors are done
        return drive.allMotors.any() { it.isBusy }
    }
}

/**
 * Phase that moves the robot to a position relative to its current location.
 *
 * This phase uses mecanum drive kinematics to calculate motor targets for moving
 * to a specified position. Coordinates are relative to the robot's current position
 * and orientation at the start of the phase.
 *
 * @param ycm Forward/backward distance in centimeters (positive = forward, negative = backward)
 * @param xcm Left/right distance in centimeters (positive = right, negative = left)
 * @param power Motor power (0.0 to 1.0) to use for movement
 *
 * @see Drivebase.goto
 */
class GoPhase(val ycm: Double, val xcm: Double, val power: Double): AutonomousPhase {
    override fun Robot.initPhase() {
        // Calculate and set motor targets for the desired movement
        drive.goto(ycm, xcm, power)
    }

    override fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean {
        // Continue the phase while any motor is still busy moving
        // Returns true to keep running, false when all motors reach their targets
        return drive.allMotors.any() { it.isBusy }
    }
}

/**
 * Example autonomous OpMode demonstrating the PhasedAutonomous framework.
 *
 * This autonomous routine performs a simple "there and back" movement:
 * 1. **Phase 1**: Drive forward 12cm at 50% power
 * 2. **Phase 2**: Turn 180 degrees (turn around) at 50% power
 * 3. **Phase 3**: Drive forward 12cm at 50% power (now moving back toward start)
 *
 * ## Key Points
 *
 * - Each phase executes sequentially - the next phase won't start until the previous one completes
 * - The framework automatically handles phase transitions
 * - The OpMode stops automatically after all phases complete
 * - Telemetry shows which phase is running and timing information
 *
 * ## Customizing This Example
 *
 * To create your own autonomous routine:
 * 1. Copy this class and rename it (e.g., `RedLeftAuto`)
 * 2. Modify the `@Autonomous` annotation to give it a display name
 * 3. Change the phases in the constructor to match your routine
 * 4. Create additional phase classes as needed for your robot's mechanisms
 *
 * Example with a custom name:
 * ```kotlin
 * @Autonomous(name = "Red Left Side", group = "Competition")
 * class RedLeftAuto : PhasedAutonomous(
 *     GoPhase(30.0, 0.0, 0.6),      // Drive forward 30cm
 *     DeployArmPhase(),              // Deploy scoring mechanism
 *     TurnPhase(-90, 0.5),           // Turn right 90 degrees
 *     GoPhase(15.0, 0.0, 0.5)        // Drive to scoring position
 * )
 * ```
 */
@Autonomous
class Autonomous : PhasedAutonomous(
    GoPhase(12.0, 0.0, 0.5),    // Move forward 12cm
    TurnPhase(180, 0.5),         // Turn around 180 degrees
    GoPhase(12.0, 0.0, 0.5)      // Move forward 12cm (back to start)
)