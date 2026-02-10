package org.beargineers.platform

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledFuture
import java.util.concurrent.TimeUnit

class PositionSnapshot(val position: Position, val timestamp: Long)

class LLPinpointLocalizer(robot: BaseRobot) : Localizer, Hardware(robot) {
    val Camera_positionTolerance by config(1.0)
    val Camera_headingTolerance by config(1.0)

    private val pinpoint by hardware<GoBildaPinpointDriver>()
    private val limelight by hardware<Limelight3A>("limelight")

    private val odometrySnapshots = ArrayDeque<PositionSnapshot>(HISTORY_SIZE)
    private val visionBuffer = ArrayDeque<PositionSnapshot>(VISION_BUFFER_SIZE)

    private var lastUpdatedLLTime = 0.0

    @Volatile
    private lateinit var pollingTask: ScheduledFuture<*>

    override fun init() {
        pinpoint.setOffsets(
            PinpointConfig.pinpoint_xOffset,
            PinpointConfig.pinpoint_yOffset,
            DistanceUnit.CM
        )

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        pinpoint.setEncoderDirections(
            PinpointConfig.pinpoint_xEncoderDirection.direction(),
            PinpointConfig.pinpoint_yEncoderDirection.direction()
        )
        pinpoint.resetPosAndIMU()

        val yawScalar = PinpointConfig.pinpoint_yawScalar
        if (yawScalar > 0.0001) {
            pinpoint.setYawScalar(yawScalar)
        }

        limelight.pipelineSwitch(0)
        limelight.start()

        pollingTask =executor.scheduleWithFixedDelay({
            val result = limelight.latestResult
            if (result.timestamp != lastUpdatedLLTime && result.isValid && result.staleness < 50) {
                lastUpdatedLLTime = result.timestamp
                val totalLatencyMs = result.staleness + result.captureLatency + result.targetingLatency + result.parseLatency
                val captureTimestamp = System.nanoTime() - (totalLatencyMs * 1_000_000).toLong()

                val position = position(result)
                if (position != null) {
                    synchronized(visionBuffer) {
                        visionBuffer.addLast(PositionSnapshot(position, captureTimestamp))
                        if (visionBuffer.size >= VISION_BUFFER_SIZE) visionBuffer.removeFirst()
                    }
                }
            }
        }, 0L, 5, TimeUnit.MILLISECONDS)
    }

    private fun PositionSnapshot.translateToCurrentMoment(): Position {
        val cp = RobotOpMode.lastKnownPosition
        for ((next, prev) in odometrySnapshots.zipWithNext()) {
            if (timestamp in prev.timestamp..next.timestamp) {
                val ratio = (timestamp - prev.timestamp).toDouble() / (next.timestamp - prev.timestamp)
                val pp = prev.position
                val np = next.position
                val estimate = Position(
                    pp.x + (np.x - pp.x) * ratio,
                    pp.y + (np.y - pp.y) * ratio,
                    (pp.heading + (np.heading - pp.heading) * ratio).normalize())

                return Position(
                    x = position.x + (cp.x - estimate.x),
                    y = position.y + (cp.y - estimate.y),
                    heading = (position.heading + cp.heading - estimate.heading).normalize(),
                )
            }
        }

        return position
    }

    private fun DcMotorSimple.Direction.direction(): GoBildaPinpointDriver.EncoderDirection {
        return if (this == DcMotorSimple.Direction.FORWARD) GoBildaPinpointDriver.EncoderDirection.FORWARD else GoBildaPinpointDriver.EncoderDirection.REVERSED
    }

    override fun loop() {
        pinpoint.update()
        limelight.updateRobotOrientation(pinpoint.getHeading(AngleUnit.DEGREES))
    }

    override fun setStartingPosition(position: Position) {
        updateCurrentPosition(position)
        updatePositionEstimate(position)
        odometrySnapshots.clear()
    }

    private fun updateCurrentPosition(cp: Position) {
        RobotOpMode.lastKnownPosition = cp
        odometrySnapshots.addFirst(PositionSnapshot(cp, System.nanoTime()))
        if (odometrySnapshots.size >= HISTORY_SIZE) {
            odometrySnapshots.removeLast()
        }
    }

    override fun update() {
        val pose = getRobotPose()
        if (pose != null) {
            telemetry.addData("Vision", "✓ acquired")
            updatePositionEstimate(pose)
            updateCurrentPosition(pose)
        }
        else {
            telemetry.addData("Vision", "✗ odometry only")
            updateCurrentPosition(getPosition())
        }
    }

    private fun updatePositionEstimate(position: Position) {
        odometrySnapshots.clear()
        with(pinpoint) {
            setPosX(position.x.cm(), DistanceUnit.CM)
            setPosY(position.y.cm(), DistanceUnit.CM)
            setHeading(position.heading.radians(), AngleUnit.RADIANS)
        }
    }

    override fun getPosition(): Position {
        with(pinpoint) {
            return Position(
                x = getPosX(DistanceUnit.CM).cm,
                y = getPosY(DistanceUnit.CM).cm,
                heading = getHeading(AngleUnit.DEGREES).degrees
            )
        }
    }

    override fun getVelocity(): RobotCentricPosition {
        return RobotCentricPosition(
            pinpoint.getVelX(DistanceUnit.CM).cm,
            pinpoint.getVelY(DistanceUnit.CM).cm,
            pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES).degrees,
        )
    }

    override fun stop() {
        limelight.stop()
        pollingTask.cancel(false)
    }

    private fun getRobotPose(): Position? {
        val normalizer = PositionNormalDistribution(5)
        synchronized(visionBuffer) {
            for (snapshot in visionBuffer) {
                normalizer.update(snapshot.translateToCurrentMoment())
            }
        }
        return normalizer.result(Camera_positionTolerance.cm, Camera_headingTolerance.degrees)
    }

    private fun position(latestResult: LLResult): Position? {
        return latestResult.botpose?.robotPose()
    }

    companion object {
        const val HISTORY_SIZE = 200
        const val VISION_BUFFER_SIZE = 20

        private val executor = Executors.newSingleThreadScheduledExecutor()
    }
}