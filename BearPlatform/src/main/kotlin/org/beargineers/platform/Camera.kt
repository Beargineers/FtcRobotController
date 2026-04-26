package org.beargineers.platform

abstract class Camera(robot: BaseRobot) : Hardware(robot), AbsoluteLocalizer {
    val CameraPosition_forward by config(0.0)
    val CameraPosition_up by config(0.0)
    val CameraPosition_right by config(0.0)
    val CameraPosition_pitch by config(0.0)
    val CameraPosition_yaw by config(0.0)
    val CameraPosition_roll by config(0.0)

    // Distance parameters in cm, angles in degrees
}