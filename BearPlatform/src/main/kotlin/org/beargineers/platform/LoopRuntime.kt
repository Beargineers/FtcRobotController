package org.beargineers.platform

import kotlinx.coroutines.CancellableContinuation
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineDispatcher
import kotlinx.coroutines.CoroutineName
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Deferred
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.async
import kotlinx.coroutines.suspendCancellableCoroutine
import java.util.ArrayDeque
import kotlin.coroutines.CoroutineContext
import kotlin.coroutines.resume

/**
 * Single-threaded, manually pumped event-loop runtime.
 *
 * You call tick() from your outer loop.
 * All coroutine execution/resumption happens only during [tick].
 */
class LoopRuntime {
    private val ready = ArrayDeque<Runnable>()
    private val nextTickWaiters = ArrayDeque<CancellableContinuation<Unit>>()
    private val uncaughtFailures = ArrayDeque<Throwable>()

    private val rootJob: Job = SupervisorJob()

    val dispatcher: CoroutineDispatcher = object : CoroutineDispatcher() {
        override fun dispatch(context: CoroutineContext, block: Runnable) {
            // enqueue for execution on this loop
            ready.offer(block)
        }
    }

    val scope: CoroutineScope = CoroutineScope(rootJob + dispatcher)

    /**
     * Resume on the next loop iteration.
     */
    suspend fun nextTick(): Unit =
        suspendCancellableCoroutine { cont ->
            nextTickWaiters.offer(cont)

            cont.invokeOnCancellation {
                cont.context[CoroutineName]?.let {
                    if (!it.name.startsWith("!")) {
                        Frame.log("Cancelling ${it.name}")
                    }
                }
                // O(n), but fine for a minimal example.
                nextTickWaiters.remove(cont)
            }
        }

    /**
     * Convenience boundary for non-coroutine callers that want a result.
     * If it fails, the exception is rethrown from the next [tick] that observes it.
     */
    fun <T> submit(name: String, block: suspend CoroutineScope.() -> T): Deferred<T> {
        val deferred = scope.async(block = block, context = CoroutineName(name))

        deferred.invokeOnCompletion { throwable ->
            if (throwable != null && throwable !is CancellationException) {
                uncaughtFailures.addLast(throwable)
            }
        }

        return deferred
    }

    /**
     * One loop iteration:
     * 1) move next-tick waiters into ready state
     * 2) run everything currently ready
     *
     * Returns number of executed runnables.
     */
    fun tick(): Int {
        rethrowPendingFailure()

        // Snapshot waiters so anything that calls nextTick() during this tick
        // resumes on the *following* tick, not this one.
        val toResume = ArrayList<CancellableContinuation<Unit>>(nextTickWaiters.size)
        while (true) {
            toResume += (nextTickWaiters.poll() ?: break)
        }

        // Resume through the continuation; execution is routed via dispatcher.
        for (cont in toResume) {
            if (cont.isActive) cont.resume(Unit)
        }

        var ran = 0
        while (true) {
            val task = ready.poll() ?: break
            task.run()
            ran++
            rethrowPendingFailure()
        }
        return ran
    }

    fun stop() {
        rootJob.cancel()
        uncaughtFailures.clear()
        nextTickWaiters.clear()
        ready.clear()
    }

    private fun rethrowPendingFailure() {
        val throwable = uncaughtFailures.pollFirst() ?: return
        throw throwable
    }
}
