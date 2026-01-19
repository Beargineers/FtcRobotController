package org.beargineers.platform

interface Part<T:Any> {
    fun get(robot: Robot): T = robot.getPart(this)
    fun build(robot: Robot): T
}