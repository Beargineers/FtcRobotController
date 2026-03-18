package org.beargineers.platform

import kotlinx.coroutines.CancellableContinuation
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.CoroutineDispatcher
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.launch
import kotlinx.coroutines.suspendCancellableCoroutine
import java.util.ArrayDeque
import kotlin.coroutines.CoroutineContext
import kotlin.coroutines.resume

/**
 * Single-threaded, manually pumped event-loop runtime.
 *
 * You call tick() from your outer loop.
 * All coroutine execution/resumption happens only during tick().
 */
class LoopRuntime {
    private val ready = ArrayDeque<Runnable>()
    private val nextTickWaiters = ArrayDeque<CancellableContinuation<Unit>>()

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
                // O(n), but fine for a minimal example.
                nextTickWaiters.remove(cont)
            }
        }

    /**
     * Convenience boundary for non-coroutine callers that want a result.
     */
    fun <T> submit(block: suspend CoroutineScope.() -> T): CompletableDeferred<T> {
        val result = CompletableDeferred<T>(rootJob)
        scope.launch {
            try {
                result.complete(block())
            } catch (t: Throwable) {
                result.completeExceptionally(t)
            }
        }
        return result
    }

    /**
     * One loop iteration:
     * 1) move next-tick waiters into ready state
     * 2) run everything currently ready
     *
     * Returns number of executed runnables.
     */
    fun tick(): Int {
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
        }
        return ran
    }

    fun stop() {
        rootJob.cancel()
        nextTickWaiters.clear()
        ready.clear()
    }
}