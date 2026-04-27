package org.beargineers.platform

import com.qualcomm.robotcore.util.ElapsedTime
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.NonCancellable
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import org.junit.Test
import kotlin.time.Duration.Companion.milliseconds
import kotlin.time.Duration.Companion.seconds

class CoroutinesLoopTest : RobotTest() {
    private inline fun <reified T : Throwable> expectThrows(block: () -> Unit): T {
        try {
            block()
        } catch (throwable: Throwable) {
            if (throwable is T) {
                return throwable
            }
            throw throwable
        }

        throw AssertionError("Expected ${T::class.simpleName}")
    }

    @Test
    fun testCoroutinesDoWhile() {
        var counter = 0
        opMode.submitJob {
            robot.cancelWhen({ counter >= 10 }) {
                while (true) {
                    counter++
                    println("At counter=$counter")
                    opMode.nextTick()
                }
            }

            robot.cancelWhen({ counter <= 5 }) {
                while (true) {
                    counter--
                    println("At counter=$counter")
                    opMode.nextTick()
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

        opMode.submitJob {
            launch {
                while (!trigger) {
                    opMode.nextTick()
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
        opMode.submitJob {
            opMode.nextTick()
            throw RuntimeException("Test exception")
        }

        val thrown = expectThrows<RuntimeException> {
            opMode.loop.tick()
            opMode.loop.tick()
        }

        assert(thrown.message == "Test exception")
    }

    @Test
    fun testSubmitRethrowsImmediateException() {
        opMode.loop.submit("") {
            error("throw me")
        }

        val thrown = expectThrows<IllegalStateException> {
            opMode.loop.tick()
        }

        assert(thrown.message == "throw me")
    }

    @Test
    fun testCancellation() {
        var startedCounter = 0
        var cancellationCounter = 0

        repeat(5) {
            val job = opMode.submitJob {
                try {
                    startedCounter++
                    opMode.nextTick()
                    error("nextTick() should throw CancellationException when the job is cancelled")
                } catch (exception: CancellationException) {
                    cancellationCounter++
                    throw exception
                }
            }

            opMode.loop.tick()
            job.cancel()
            opMode.loop.tick()

            assert(job.isCancelled)
        }

        assert(startedCounter == 5)
        assert(cancellationCounter == 5)
    }

    @Test
    fun testLaunchingInCancelledContext() {
        var worked = false
        val job = opMode.submitJob {
            try {
                opMode.nextTick()
                error("nextTick() should throw CancellationException when the job is cancelled")
            } finally {
                withContext(NonCancellable) {
                    coroutineScope {
                        launch {
                            worked = true
                        }
                    }
                }
            }
        }

        opMode.loop.tick()
        job.cancel()

        repeat(10) {opMode.loop.tick()}

        assert(worked)
    }

    @Test
    fun testLaunchingInCancelledContext2() {
        var worked = false
        val job = opMode.submitJob {
            try {
                opMode.nextTick()
                error("nextTick() should throw CancellationException when the job is cancelled")
            } finally {
                opMode.submitJob {
                    worked = true
                }
            }
        }

        opMode.loop.tick()
        job.cancel()

        repeat(10) {opMode.loop.tick()}

        assert(worked)
    }
}
