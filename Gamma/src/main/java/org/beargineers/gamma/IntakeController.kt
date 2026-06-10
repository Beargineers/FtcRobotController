package org.beargineers.gamma

import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import org.beargineers.platform.Frame
import org.beargineers.platform.decode.IntakeMode
import org.beargineers.platform.decode.intakeMode
import org.beargineers.platform.submitJob
import kotlin.time.Duration

class IntakeController(private val bot: GammaRobot) {
    private enum class Request(val priority: Int) {
        BASE(0),
        CAPACITY_LIMIT(1),
        SHOOTER(2),
        ABORT_CLEAR(3)
    }

    private val lock = Any()
    private val requests = mutableMapOf<Request, IntakeMode>()
    private var reverseJob: Job? = null
    private var revision = 0L

    fun revision(): Long = synchronized(lock) { revision }

    fun setBaseMode(mode: IntakeMode) {
        setRequest(Request.BASE, mode)
    }

    fun setShooterMode(mode: IntakeMode?) {
        setRequest(Request.SHOOTER, mode)
    }

    fun setCapacityLimited(limited: Boolean) {
        setRequest(Request.CAPACITY_LIMIT, if (limited) IntakeMode.OFF else null)
    }

    fun setModeOverrideCapacity(mode: IntakeMode) {
        setCapacityLimited(false)
        setBaseMode(mode)
    }

    fun reverseFor(duration: Duration) {
        reverseJob?.cancel()
        setRequest(Request.ABORT_CLEAR, IntakeMode.REVERSE)
        reverseJob = bot.submitJob("Reverse intake to clear aborted shot") {
            delay(duration)
            clearRequest(Request.ABORT_CLEAR)
        }
    }

    private fun setRequest(request: Request, mode: IntakeMode?) {
        val resolved = synchronized(lock) {
            val previous = requests[request]
            if (previous == mode) {
                return
            }

            if (mode == null) {
                requests.remove(request)
            } else {
                requests[request] = mode
            }

            revision++
            resolveModeLocked()
        }

        applyResolvedMode(resolved)
    }

    private fun clearRequest(request: Request) {
        setRequest(request, null)
    }

    private fun resolveModeLocked(): IntakeMode {
        return requests.maxByOrNull { it.key.priority }?.value ?: IntakeMode.OFF
    }

    private fun applyResolvedMode(mode: IntakeMode) {
        if (bot.intakeMode == mode) {
            return
        }

        Frame.log("Intake controller resolved mode: $mode")
        bot.intakeMode = mode
    }
}
