package org.beargineers.platform

import org.beargineers.platform.decode.DecodeRobot

class TestOpMode : RobotOpMode<Robot>() {
    override val alliance: Alliance = Alliance.RED

    override suspend fun Robot.autoProgram() {
        // Do nothing by default
    }
}

class TestRobot(override val opMode: TestOpMode) : DecodeRobot {
    val localizer = TestLocalizer()

    override suspend fun shoot(holdPosition: Boolean) {
        TODO("Not yet implemented")
    }

    override suspend fun prepareForShooting() {
        TODO("Not yet implemented")
    }

    override fun isShooting(): Boolean {
        TODO("Not yet implemented")
    }

    override fun adjustShooting(distance: Double, angle: Double) {
        TODO("Not yet implemented")
    }

    override val shootingAngleCorrection: Angle
        get() = 0.degrees
    override val artifactsCount: Int
        get() = 0

    override var hasTurret = false

    val parts = mutableMapOf<Part<*>, Any>()
    @Suppress("UNCHECKED_CAST")
    override fun <T:Any> getPart(part: Part<T>): T {
        return parts.getOrPut(part) {
            part.build(this)
        } as T
    }

    override fun assumePosition(position: Position) {
        localizer.setStartingPosition(position)
    }

    override fun stop() {
        TODO("Not yet implemented")
    }

    override fun init() {
        TODO("Not yet implemented")
    }

    override fun start() {
        TODO("Not yet implemented")
    }

    override fun loop() {
        TODO("Not yet implemented")
    }

    override val currentPosition: Position
        get() = localizer.currentPosition
    override val currentVelocity: Position
        get() = localizer.getVelocity()

    override fun predictedPosition(nTicks: Int): Position {
        return currentPosition
    }
}

class TestLocalizer : Localizer {
    var currentPosition: Position = Position.ZERO
    override fun setStartingPosition(position: Position) {
        currentPosition = position
    }

    override fun update() {
    }

    override fun getPosition(): Position {
        return currentPosition
    }

    override fun getVelocity(): Position {
        return Position.ZERO
    }
}
