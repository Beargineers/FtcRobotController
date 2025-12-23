@file:Suppress("unused")

package org.beargineers.platform

/**
 * Example demonstrating how to use the spline path following system.
 *
 * This file shows various usage patterns for the followSplinePath function
 * in different autonomous scenarios.
 */

/**
 * Example 1: Simple path with waypoints
 *
 * Creates a smooth path through multiple waypoints to reach a target.
 * The robot will accelerate smoothly, navigate through the waypoints,
 * and decelerate to the target position.
 * Each waypoint specifies both position and heading for precise control.
 */
fun BaseRobot.exampleSimplePath() {
    // Define waypoints with positions and headings
    val waypoints = listOf(
        Location(30.cm, 20.cm).withHeading(0.degrees),      // Face forward
        Location(60.cm, 50.cm).withHeading(45.degrees),     // Start turning
        Location(90.cm, 40.cm).withHeading(90.degrees)      // Face sideways
    )

    // Define target with final heading
    val target = Location(120.cm, 80.cm).withHeading(45.degrees)

    // Follow the path (call this in your loop function)
    if (followSplinePath(target, waypoints, maxSpeed = 0.8)) {
        telemetry.addLine("Following path...")
    } else {
        telemetry.addLine("Target reached!")
    }
}

/**
 * Example 2: Direct path without waypoints
 *
 * Creates a smooth direct path to the target.
 * Even without waypoints, the system provides smooth acceleration/deceleration.
 */
fun BaseRobot.exampleDirectPath() {
    val target = Location(100.cm, 50.cm).withHeading(0.degrees)

    // No waypoints - direct smooth path
    if (followSplinePath(target, emptyList(), maxSpeed = 1.0)) {
        telemetry.addLine("Driving directly to target...")
    } else {
        telemetry.addLine("Arrived!")
        stopDriving()
    }
}

/**
 * Example 3: Fine-tuning path parameters
 *
 * Shows how to adjust path following parameters for different scenarios.
 */
fun BaseRobot.exampleParameterTuning() {
    // For tight spaces - reduce speed and lookahead
    // Modify in config file or:
    // lookaheadDistance = 15.0 (smaller = tighter following)
    // maxPathAcceleration = 30.0 (lower = gentler)
    // maxPathDeceleration = 40.0

    // For open field - increase speed and lookahead
    // lookaheadDistance = 30.0 (larger = smoother but less precise)
    // maxPathAcceleration = 80.0 (higher = more aggressive)
    // maxPathDeceleration = 100.0

    val waypoints = listOf(
        Location(50.cm, 25.cm).withHeading(45.degrees)  // Turn while moving
    )
    val target = Location(100.cm, 50.cm).withHeading(0.degrees)

    followSplinePath(target, waypoints, maxSpeed = 0.7)
}

/**
 * Example 4: Emergency stop during path following
 */
fun BaseRobot.exampleEmergencyStop() {
    // Start following a path
    val target = Location(100.cm, 100.cm).withHeading(0.degrees)
    val waypoints = listOf(
        Location(50.cm, 50.cm).withHeading(45.degrees)
    )

    if (followSplinePath(target, waypoints)) {
        // Check for emergency condition
        // if (someEmergencyCondition) {
        //     stopFollowingPath()
        //     telemetry.addLine("Emergency stop!")
        // }
    }
}

/**
 * Example 5: Precise heading control along the path
 *
 * Demonstrates using waypoint headings to control robot orientation
 * throughout the path - useful for intake positioning, vision tracking, etc.
 */
fun BaseRobot.examplePreciseHeadingControl() {
    // Robot faces forward, then rotates to face target while moving
    val waypoints = listOf(
        Location(30.cm, 10.cm).withHeading(0.degrees),      // Face forward
        Location(60.cm, 30.cm).withHeading(90.degrees),     // Face right
        Location(90.cm, 50.cm).withHeading(180.degrees)     // Face backward
    )

    val target = Location(120.cm, 70.cm).withHeading(270.degrees)  // Face left at end

    if (followSplinePath(target, waypoints, maxSpeed = 0.6)) {
        telemetry.addLine("Rotating smoothly while following path...")
    }
}

/**
 * Configuration tips:
 *
 * Add these to your config.properties file to tune the path following:
 *
 * # Lookahead distance (cm) - how far ahead the robot looks
 * # Smaller = tighter following, Larger = smoother but less precise
 * lookaheadDistance=20.0
 *
 * # Maximum acceleration/deceleration (cm/s²)
 * # Higher = more aggressive, Lower = smoother
 * maxPathAcceleration=50.0
 * maxPathDeceleration=60.0
 *
 * # Curvature speed factor (cm)
 * # Effective turn radius at which to reduce speed
 * # Smaller = slow down more on curves
 * curvatureSpeedFactor=30.0
 *
 * # Standard drive tolerances still apply
 * positionTolerance=2.0
 * headingTolerance=5.0
 */
