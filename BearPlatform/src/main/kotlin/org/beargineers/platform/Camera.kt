package org.beargineers.platform

abstract class Camera(robot: BaseRobot) : Hardware(robot), AbsoluteLocalizer {
    // Distance parameters in cm, angles in degrees
    val CameraPosition_forward by robot.config(0.0)
    val CameraPosition_up by robot.config(0.0)
    val CameraPosition_right by robot.config(0.0)
    val CameraPosition_pitch by robot.config(0.0)
    val CameraPosition_yaw by robot.config(0.0)
    val CameraPosition_roll by robot.config(0.0)

    val Camera_positionTolerance by robot.config(1.0)
    val Camera_headingTolerance by robot.config(1.0)
}