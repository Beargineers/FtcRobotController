# BearPlatform Framework

Reusable robotics framework for FTC robots. Provides hardware abstraction, position tracking, autonomous phases, and OpMode base classes.

## Core Classes

### Hardware

Base class for hardware subsystems. Provides lazy hardware initialization via property delegates.

```kotlin
class MyDrivetrain(robot: BaseRobot) : Hardware(robot) {
    val lf: DcMotor by hardware("leftFront")  // Custom hardware name
    val rf: DcMotor by hardware()             // Uses property name "rf"
    val imu: IMU by hardware()

    override fun init() {
        // Initialize subsystem
    }

    override fun loop() {
        // Called each loop cycle
    }
}
```

The `hardware()` delegate:
- Retrieves hardware from `hardwareMap` lazily on first access
- Uses property name as hardware config name by default
- Accepts optional parameter to override hardware name
- Automatically registers subsystem with robot for lifecycle management

`setMotorPower()` method provides voltage compensation for consistent motor behavior across battery voltages.

### BaseRobot

Abstract base class for robot implementations. Manages hardware subsystems, position tracking, and autonomous navigation.

```kotlin
abstract class MyRobot(opMode: RobotOpMode<*>) : BaseRobot(opMode) {
    override val drive: Drivetrain = MyDrivetrain(this)
    val intake = MyIntake(this)
    val arm = MyArm(this)
}
```

Responsibilities:
- Manages hardware subsystem lifecycle (init, loop, stop)
- Tracks robot position on field using encoder odometry
- Provides `driveToTarget()` for proportional control navigation
- Updates telemetry and field visualization

Position tracking integrates encoder-based odometry with IMU heading using mecanum wheel kinematics.

### RobotOpMode

Base class for TeleOp and simple autonomous OpModes.

```kotlin
@TeleOp(name = "Main TeleOp")
class MainTeleOp : RobotOpMode<MyRobot>(Alliance.RED) {
    override fun createRobot(opMode: RobotOpMode<MyRobot>) = MyRobot(opMode)

    override fun loop() {
        // TeleOp control logic
        robot.drive.drive(
            forwardPower = -gamepad1.left_stick_y.toDouble(),
            rightPower = gamepad1.left_stick_x.toDouble(),
            turnPower = gamepad1.right_stick_x.toDouble()
        )
    }
}
```

Features:
- Generic robot type parameter for type-safe access
- Alliance parameter (RED/BLUE)
- Automatic robot initialization
- Button helper methods for gamepad input

Button system:
```kotlin
button(gamepad1::a) {
    // Executes once on button release
    intake.toggle()
}

toggleButton("intake", gamepad1::b) { isOn ->
    // Executes on each toggle with current state
    intake.setPower(if (isOn) 1.0 else 0.0)
}
```

### Drivetrain

Interface for drive subsystems. Extends `RelativeLocalizer`. Implementations must provide:
- `drive(forwardPower, rightPower, turnPower, slow)` - Apply drive power
- `stop()` - Stop all drive motors
- `getMovementDelta()` - Return movement since last call for odometry (from RelativeLocalizer)

Typical implementation uses mecanum wheels with encoders and IMU.

### Localizer Interfaces

**RelativeLocalizer** - Provides relative positioning (movement deltas):
- `getMovementDelta()` - Returns `RobotMovement` with forward, right, and turn changes since last query
- Used for odometry-based positioning (encoders, dead wheels, etc.)
- Accumulates errors over time but always available

**AbsoluteLocalizer** - Provides absolute positioning (field coordinates):
- `getRobotPose()` - Returns `AbsolutePose?` with current x, y, heading, and confidence on field
- `AbsolutePose` contains:
  - `pose: Position` - The robot's position on the field
  - `confidence: Double` - Quality metric (0.0 to 1.0) indicating measurement reliability
- Used for vision-based or external reference positioning (AprilTags, GPS, etc.)
- No error accumulation but may be intermittent (returns null when unavailable)

**Confidence Score Interpretation:**
- **1.0**: Excellent - Close range, perpendicular view, clear detection
- **0.7-0.9**: Good - Moderate range/angle, suitable for most uses
- **0.4-0.6**: Fair - Far range or poor angle, use with caution
- **0.0-0.3**: Poor - Very far, extreme angle, or ambiguous detection

**Confidence for Sensor Fusion:**
```kotlin
// Example: Use absolute positioning only when confidence is high
val absolutePose = aprilTagWebcam.getRobotPose()
if (absolutePose != null && absolutePose.confidence > 0.7) {
    // Trust the absolute position - reset odometry drift
    currentPosition = absolutePose.pose
} else {
    // Fall back to relative positioning
    currentPosition += drivebase.getMovementDelta()
}
```

### PhasedAutonomous

Framework for phase-based autonomous routines. See [PhasedAutonomous.kt](PhasedAutonomous.kt) for comprehensive documentation.

```kotlin
@Autonomous(name = "Red South")
class RedSouth : PhasedAutonomous<MyRobot>(Alliance.RED) {
    override fun createRobot(opMode: RobotOpMode<MyRobot>) = MyRobot(opMode)

    override fun PhaseBuilder<MyRobot>.createPhases() {
        assumePosition(START_POSITION)

        driveTo(SPIKE_LEFT)
        action { intake.setPower(1.0) }
        wait(1.seconds)

        seq("Score") {
            driveTo(CHAMBER)
            action { arm.deploy() }
            wait(0.5.seconds)
        }

        par("Deploy and intake") {
            action { arm.lower() }
            action { intake.start() }
            driveTo(PICKUP_POSITION)
        }

        driveTo(OBSERVATION_ZONE)
    }
}
```

Key concepts:
- **AutonomousPhase** - Interface for custom phases with `initPhase()` and `loopPhase()`
- **SequentialPhase** - Container for executing phases one after another
- **ParallelPhase** - Container for executing phases concurrently
- **DSL functions** - `driveTo()`, `wait()`, `action()`, `composite()`, `parallel()`, `driveRelative()`
- Built-in phases: WaitPhase, SimpleActionPhase, GotoPosePhase, DriveRelative

## Coordinate System

### Position

Represents position and heading on the field.

```kotlin
data class Position(
    x: Double,
    y: Double,
    heading: Double,
    distanceUnit: DistanceUnit,
    angleUnit: AngleUnit
)
```

Field coordinate system (FTC standard):
- Origin at field center (0, 0)
- X-axis toward audience (positive = audience side)
- Y-axis toward red alliance wall (positive = red side)
- Heading: 0° facing forward (positive X), increases counter-clockwise

Methods:
- `toDistanceUnit()`, `toAngleUnit()` - Unit conversion
- `rotate()`, `shift()` - Transform position
- `plus()`, `minus()` - Position arithmetic
- `normalizeHeading()` - Normalize heading to [-180°, 180°] or [-π, π]

### Location

Position without heading. Use `withHeading()` to create Position.

### RobotMovement

Represents relative movement in robot frame (forward, right, turn).

```kotlin
data class RobotMovement(
    forward: Double,
    right: Double,
    turn: Double,
    distanceUnit: DistanceUnit,
    angleUnit: AngleUnit
)
```

Used for odometry and relative navigation. Companion object provides factory methods: `forwardInch()`, `rightInch()`, `turnCCW()`, `zero()`.

## Alliance System

```kotlin
enum class Alliance {
    RED, BLUE
}
```

Passed to RobotOpMode constructor. Used for field position mirroring and alliance-specific logic.


Parameters adjustable from Driver Station without rebuilding.

## Usage Pattern

1. Create Hardware subclasses for each subsystem
2. Create Robot class extending BaseRobot
3. Implement Drivetrain interface for your drive subsystem
4. Create TeleOp extending RobotOpMode
5. Create autonomous routines extending PhasedAutonomous
