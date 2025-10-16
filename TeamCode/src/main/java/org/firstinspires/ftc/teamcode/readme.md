## TeamCode Module

Welcome!

This module contains the code for your team's robot controller. This implementation demonstrates how to write FTC OpModes using Kotlin with a modular hardware class pattern.

## Code Structure

The codebase uses Kotlin to provide a cleaner, more concise way to organize robot code:

### Hardware Abstraction with Delegates

The `Robot.kt` class demonstrates how to use Kotlin property delegates for hardware initialization:

```kotlin
abstract class Robot() : RobotOpModeBase() {
    val mtr : DcMotor by hardware()

    override fun init() {
        mtr.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }
}
```

The `hardware()` delegate (defined in `internal/OpModes.kt`) automatically retrieves hardware from the `hardwareMap` using the property name. You can optionally specify a custom hardware name:

```kotlin
val leftMotor: DcMotor by hardware("left_drive")  // Uses custom name
val rightMotor: DcMotor by hardware()              // Uses property name "rightMotor"
```

### Benefits of This Approach

1. **Lazy Initialization**: Hardware is only retrieved when first accessed, ensuring `hardwareMap` is ready
2. **Type Safety**: Kotlin's reified generics provide compile-time type checking
3. **Concise Syntax**: No need to repeatedly call `hardwareMap.get()`
4. **Automatic Naming**: Property names automatically map to hardware configuration names

### Creating OpModes

OpModes extend the `Robot` class and only need to implement their specific logic:

**TeleOp Example** (`Teleop.kt`):
```kotlin
@TeleOp
class Teleop : Robot() {
    override fun loop() {
        mtr.power = gamepad1.left_stick_y.toDouble()

        telemetry.addData("Elapsed", "$elapsed")
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y)
    }
}
```

**Autonomous Example** (`Autonomous.kt`):
```kotlin
@Autonomous
class Autonomous() : Robot() {
    override fun loop() {
        mtr.power = (30 - elapsed.seconds()) / 30 // Start at full speed and slowly decrease    
        telemetry.addData("Elapsed", "$elapsed")
    }
}
```

### Built-in Features

The `RobotOpModeBase` class provides:
- `elapsed`: An `ElapsedTime` timer automatically reset when the OpMode starts
- `hardware<T>()`: Property delegate for hardware initialization

## Getting Started

1. Define your robot's hardware in the `Robot` class using property delegates
2. Initialize hardware settings in `Robot.init()`
3. Create TeleOp and Autonomous OpModes by extending `Robot`
4. Implement the `loop()` method with your specific logic