# FTC Robot Controller - Beargineers #27628

Robot control software for FIRST Tech Challenge team Beargineers (#27628).


Important links and scripts. All require your computer be connected to the robot's WiFi:
- http://192.168.43.1:8080 Control Hub management website. Change WiFi name, password, 2G/5G band there. https://ftc-docs.firstinspires.org/en/latest/programming_resources/shared/managing_control_hub/Managing-a-Control-Hub.html
- http://192.168.43.1:8001 Panels server. Displays robot on the field
- http://192.168.43.1:5801 Limelight dashboard. Provides way to tune the camera
- http://192.168.43.1:5800 Limelight camera stream
- http://192.168.43.1:9000 Settings server. You may change settings here via webUI. Alternatively (easier), use updateConfig.sh
- updateConfig.sh          Upload changed config.properties file to the robot. 
- connect.sh               Connects Android Studio to robot's device so you can run/debug your program there. After executing wait a 
                           few seconds until device chooser in Android Studio shows REV...

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

### Alpha, Beta, Straffer
Team (and robot) -specific code built on BearPlatform:
- Robot hardware configuration
- Autonomous routines
- TeleOp modes
- Game-specific subsystems

See [TeamCode documentation](TeamCode/src/main/java/org/beargineers/readme.md) for implementation guide.

## Requirements

- Android Studio Ladybug (2024.2) or later
- Kotlin support enabled
