## TeamCode Module

Team-specific robot code for the Beargineers FTC team, built on the BearPlatform framework.

## Project Structure

**BearPlatform** - Reusable framework module
- Base classes: `BaseRobot`, `RobotOpMode`, `Hardware`
- Phased autonomous system with composite pattern
- Position tracking and odometry
- Alliance system (Red/Blue)

**TeamCode** - Team-specific implementations
- Robot hardware configuration
- Autonomous routines
- TeleOp modes
- Field position definitions

## Core Architecture Concepts

### Hardware

Base class for hardware subsystems. Provides the `hardware()` delegate for accessing FTC hardware.

```kotlin
class Drivebase(op: OpMode) : Hardware(op) {
    val lf: DcMotor by hardware("leftFront")  // Custom name
    val imu: IMU by hardware()                 // Uses property name "imu"
}
```

The `hardware()` delegate:
- Retrieves hardware from `hardwareMap` lazily
- Uses property name as hardware config name by default
- Accepts optional parameter to override name

### BaseRobot

Abstract base for all robots. Manages hardware subsystems and position tracking.

```kotlin
abstract class BaseRobot(op: OpMode) {
    val drive: Drivebase
    var currentPosition: Position

    fun driveToTarget(target: Position, maxSpeed: Double): Boolean
}
```

Responsibilities:
- Initialize hardware subsystems
- Track robot position on field
- Provide navigation methods

### RobotOpMode

Base class for TeleOp and simple autonomous OpModes.

```kotlin
abstract class RobotOpMode<Robot: BaseRobot>(alliance: Alliance) : OpMode() {
    lateinit var robot: Robot

    abstract override fun loop()
}
```

Provides:
- Robot instance typed to your specific robot class
- Alliance information (RED/BLUE)
- Standard OpMode lifecycle

### AutonomousPhase

Interface for defining autonomous phases. Generic over robot type.

```kotlin
interface AutonomousPhase<in Robot: BaseRobot> {
    fun Robot.initPhase()                           // Called once when phase starts
    fun Robot.loopPhase(phaseTime: ElapsedTime): Boolean  // Called each loop
}
```

Return values from `loopPhase()`:
- `true` - Continue phase
- `false` - Phase complete, move to next

Built-in phases:
- `WaitPhase` - Wait for duration
- `SimpleActionPhase` - Execute action immediately
- `GotoPosePhase` - Drive to field position
- `DriveRelative` - Move relative to current position

### CompositePhase

Container for sequential phases. Implements composite pattern.

```kotlin
class CompositePhase<Robot: BaseRobot>(
    name: String,
    phases: List<AutonomousPhase<Robot>>
)
```

Manages child phase lifecycle automatically.

### PhasedAutonomous

Base class for phase-based autonomous OpModes.

```kotlin
abstract class PhasedAutonomous<Robot: BaseRobot>(alliance: Alliance) : RobotOpMode<Robot> {
    abstract fun PhaseBuilder<Robot>.createPhases()
}
```

Override `createPhases()` to define your autonomous routine using the DSL.

### Position

Represents a position on the field.

```kotlin
data class Position(
    x: Double,
    y: Double,
    heading: Double,
    distanceUnit: DistanceUnit,
    angleUnit: AngleUnit
)
```

Field coordinate system follows FTC standard:
- Origin at field center
- X-axis toward audience
- Y-axis toward red alliance wall
- Heading: 0° facing forward, increases counter-clockwise

## Creating Your Robot

### Step 1: Define Hardware Subsystems

Create `Hardware` subclasses for each subsystem:

```kotlin
class MyDrivebase(op: OpMode) : Hardware(op) {
    val lf: DcMotor by hardware("leftFront")
    val rf: DcMotor by hardware("rightFront")
    val lb: DcMotor by hardware("leftBack")
    val rb: DcMotor by hardware("rightBack")
    val imu: IMU by hardware("imu")

    fun drive(forward: Double, right: Double, turn: Double) { /* ... */ }
}

class MyIntake(op: OpMode) : Hardware(op) {
    val motor: DcMotor by hardware("intake")
    val sensor: ColorSensor by hardware()

    fun setPower(power: Double) { motor.power = power }
}
```

### Step 2: Create Robot Class

Extend `BaseRobot` and instantiate your hardware subsystems:

```kotlin
class MyRobot(op: OpMode, alliance: Alliance) : BaseRobot(op, alliance) {
    override val drive = MyDrivebase(op)
    val intake = MyIntake(op)
    val arm = MyArm(op)
}
```

### Step 3: Define Field Positions

Create constants for field positions:

```kotlin
object FieldPositions {
    val START_RED_SOUTH = Position(
        x = -60.0, y = -60.0, heading = 90.0,
        distanceUnit = DistanceUnit.CM,
        angleUnit = AngleUnit.DEGREES
    )

    val SPIKE_LEFT = Position(/* ... */)
    val CHAMBER = Position(/* ... */)
}
```

### Step 4: Create TeleOp

```kotlin
@TeleOp(name = "Main TeleOp")
class MainTeleOp : RobotOpMode<MyRobot>(Alliance.RED) {
    override fun loop() {
        robot.drive.drive(
            forward = -gamepad1.left_stick_y.toDouble(),
            right = gamepad1.left_stick_x.toDouble(),
            turn = gamepad1.right_stick_x.toDouble()
        )
    }
}
```

### Step 5: Create Autonomous

```kotlin
@Autonomous(name = "Red South")
class RedSouth : PhasedAutonomous<MyRobot>(Alliance.RED) {
    override fun PhaseBuilder<MyRobot>.createPhases() {
        assumePosition(START_RED_SOUTH)

        driveTo(SPIKE_LEFT)
        action { robot.intake.setPower(1.0) }
        wait(1.seconds)

        composite("Score") {
            driveTo(CHAMBER)
            action { robot.arm.deploy() }
            wait(0.5.seconds)
        }
    }
}
```

## DSL Functions Reference

Available in `PhaseBuilder<Robot>.createPhases()`:

- `phase(AutonomousPhase<Robot>)` - Add a phase
- `composite(name) { ... }` - Group phases
- `driveTo(position, maxSpeed = 1.0)` - Drive to field position
- `driveRelative(movement, maxSpeed = 0.5)` - Drive relative to current position
- `wait(duration)` - Wait for duration (e.g., `1.seconds`)
- `action { ... }` - Execute immediate action
- `assumePosition(position)` - Set starting position

## Configuration Parameters

Mark objects with `@Configurable` for runtime tuning:

```kotlin
@Configurable
object DriveParams {
    var MAX_SPEED = 0.8
    var TURN_GAIN = 0.02
}
```

Access in code:
```kotlin
driveTo(target, maxSpeed = DriveParams.MAX_SPEED)
```

Adjust from Driver Station without rebuilding.