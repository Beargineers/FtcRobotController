# FTC Robot Controller - Beargineers #27628

Robot control software for FIRST Tech Challenge team Beargineers (#27628).

## Project Structure

### FtcRobotController
The official FTC SDK module. Contains the Robot Controller app and core FTC libraries. Generally not modified.

### BearPlatform
Reusable robotics framework library providing:
- Hardware abstraction with property delegates
- Base robot class with position tracking and navigation
- Phased autonomous system with composite pattern
- OpMode base classes for TeleOp and Autonomous
- Alliance system for field position mirroring

See [BearPlatform documentation](BearPlatform/src/main/kotlin/org/beargineers/platform/README.md) for architecture details.

### TeamCode
Team-specific robot code built on BearPlatform:
- Robot hardware configuration
- Autonomous routines
- TeleOp modes
- Game-specific subsystems

See [TeamCode documentation](TeamCode/src/main/java/org/beargineers/readme.md) for implementation guide.

## Requirements

- Android Studio Ladybug (2024.2) or later
- Kotlin support enabled
