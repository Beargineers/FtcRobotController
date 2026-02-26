package org.beargineers.platform

import com.qualcomm.robotcore.util.ElapsedTime
import junit.framework.TestCase.assertEquals
import org.junit.Test
import kotlin.time.Duration.Companion.milliseconds

class PhasesTest : RobotTest() {
    @Test
    fun testPar() {
        val log = StringBuilder()
        val phase = build {
            par("Parallel") {
                seq("") {
                    wait(100.milliseconds)
                    doOnce {
                        log.append("1")
                    }
                }
                doOnce {
                    log.append("2")
                }
            }
        }

        val time = ElapsedTime()
        with(phase) {
            robot.initPhase()
            while (true) {
                if (!robot.loopPhase(time)) break
            }
        }

        assertEquals("21", log.toString())
    }


    fun build(b: PhaseBuilder<Robot>.() -> Unit): AutonomousPhase<Robot> {
        val builder = PhaseBuilder(TestOpMode())
        with(builder) {
            b()
        }
        return SequentialPhase("Plan", builder.build())
    }
}