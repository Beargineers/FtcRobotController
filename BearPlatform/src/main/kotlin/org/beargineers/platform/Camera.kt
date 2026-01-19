package org.beargineers.platform

abstract class Camera(robot: BaseRobot) : Hardware(robot), AbsoluteLocalizer {
    // Distance parameters in cm, angles in degrees
    val CameraPosition_forward by config(0.0)
    val CameraPosition_up by config(0.0)
    val CameraPosition_right by config(0.0)
    val CameraPosition_pitch by config(0.0)
    val CameraPosition_yaw by config(0.0)
    val CameraPosition_roll by config(0.0)

    val Camera_positionTolerance by config(1.0)
    val Camera_headingTolerance by config(1.0)
}