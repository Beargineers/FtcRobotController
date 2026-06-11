package org.beargineers.platform

import kotlinx.coroutines.delay
import kotlin.time.Duration.Companion.milliseconds


suspend fun Robot.executeCommands(commands: String) {
    val cmdStrings = commands.split('|').map { it.trim() }
    var currentSpeed = 1.0
    var lastPosition = currentPosition
    var currentPath = mutableListOf<Waypoint>()

    suspend fun drivePath() {
        if (currentPath.isNotEmpty()) {
            drivePath(currentPath, applyMirroring = true)
            currentPath.clear()
        }
    }

    for (cmd in cmdStrings) {
        when {
            cmd.startsWith("speed") -> {
                currentSpeed = cmd.substringAfter("speed").trim().toDouble()
            }

            cmd.startsWith("wait") -> {
                drivePath()
                val waitTime = cmd.substringAfter("wait").trim().toLong()
                delay(waitTime.milliseconds)
            }

            cmd.startsWith("gorel") -> {
                val pos = Position.parse(cmd.substringAfter("gorel").trim())
                lastPosition = lastPosition.shift(pos.x, pos.y).rotate(pos.heading)
                currentPath += pathTo(lastPosition, currentSpeed)
            }

            cmd.startsWith("go") -> {
                lastPosition = Position.parse(cmd.substringAfter("go").trim())
                currentPath += pathTo(lastPosition, currentSpeed)
            }
        }
    }

    drivePath()
}