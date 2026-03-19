package org.beargineers.platform

import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.junit.Test
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

class CoroutinesLoopTest : RobotTest() {
    @Test
    fun testCoroutinesDoWhile() {
        var counter = 0
        opMode.loop.submit {
            robot.doWhile({ counter < 10 }) {
                while (true) {
                    counter++
                    println("At counter=$counter")
                    opMode.loop.nextTick()
                }
            }

            robot.doWhile({ counter > 5 }) {
                while (true) {
                    counter--
                    println("At counter=$counter")
                    opMode.loop.nextTick()
                }
            }
            println("Done")
        }

        repeat(100) {
            opMode.loop.tick()
        }

        assert(counter == 5)
    }

    @Test
    fun testChildTask() {
        var fuse = false
        var trigger = false
        var complete = false

        opMode.loop.submit {
            launch {
                while (!trigger) {
                    opMode.loop.nextTick()
                }
                complete = true
            }

            coroutineScope {
                val child = launch {
                    println("In child before delay")
                    delay(100.seconds)
                    println("In child after delay")
                    fuse = true
                }

                delay(50.milliseconds)
                child.cancel()
                trigger = true
                delay(10.milliseconds)
            }
        }

        val timer = ElapsedTime()
        while (timer.milliseconds() < 500) {
            opMode.loop.tick()
        }

        assert(fuse == false)
        assert(complete == true)
    }

    @Test
    fun testExceptionHandling() {
        var success = false
        opMode.loop.submit {
            throw RuntimeException("Test exception")
        }.invokeOnCompletion { throwable ->
            if (throwable?.message == "Test exception") {
                success = true
            }
        }
        repeat(5) {opMode.loop.tick()}
        assert(success)
    }
}