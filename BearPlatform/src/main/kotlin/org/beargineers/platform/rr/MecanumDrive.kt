package org.beargineers.platform.rr

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.IdentityPoseMap
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Pose2dDual
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.VelConstraint
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.acmerobotics.roadrunner.ftc.FlightRecorder.write
import com.acmerobotics.roadrunner.ftc.throwIfModulesAreOutdated
import com.acmerobotics.roadrunner.now
import com.acmerobotics.roadrunner.range
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Position
import org.beargineers.platform.RobotCentricPosition
import org.beargineers.platform.rr.messages.DriveCommandMessage
import org.beargineers.platform.rr.messages.MecanumCommandMessage
import org.beargineers.platform.rr.messages.PoseMessage
import java.util.ArrayDeque
import kotlin.math.ceil
import kotlin.math.max

interface SimpleLocalizer {
    val currentPosition: Position
    val currentVelocity: RobotCentricPosition
}

class SimpleRobotLocalizer(val robot: BaseRobot) : SimpleLocalizer {
    override val currentPosition: Position
        get() = robot.currentPosition
    override val currentVelocity: RobotCentricPosition
        get() = robot.currentVelocity
}

class MecanumDrive(val hardwareMap: HardwareMap, val localizer: SimpleLocalizer) {
    constructor(robot: BaseRobot) : this(robot.opMode.hardwareMap, SimpleRobotLocalizer(robot))

    class Params {
        // drive model parameters
        
        var inPerTick: Double = 0.001986031578
        var lateralInPerTick: Double = 0.001393583788980037
        var trackWidthTicks: Double = 6089.591997375772

        // feedforward parameters (in tick units)
        var kS: Double = 1.1521644029631526
        var kV: Double = 0.000242986196242149
        var kA: Double = 0.000057

        // path profile parameters (in inches)
        var maxWheelVel: Double = 50.0
        var minProfileAccel: Double = -30.0
        var maxProfileAccel: Double = 50.0

        // turn profile parameters (in radians)
        var maxAngVel: Double = Math.PI // shared with path
        var maxAngAccel: Double = Math.PI

        // path controller gains
        var axialGain: Double = 25.0
        var lateralGain: Double = 20.0
        var headingGain: Double = 18.0 // shared with turn

        var axialVelGain: Double = 1.5
        var lateralVelGain: Double = 0.05
        var headingVelGain: Double = 0.0 // shared with turn
    }

    private val kinematics = MecanumKinematics(
        PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick
    )

    private val defaultTurnConstraints = TurnConstraints(
        PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel
    )
    private val defaultVelConstraint: VelConstraint = MinVelConstraint(
        listOf(
            kinematics.WheelVelConstraint(PARAMS.maxWheelVel),
            AngularVelConstraint(PARAMS.maxAngVel)
        )
    )
    private val defaultAccelConstraint: AccelConstraint =
        ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel)

    
    val leftFront: DcMotorEx
    val leftBack: DcMotorEx
    val rightBack: DcMotorEx
    val rightFront: DcMotorEx

    
    val voltageSensor: VoltageSensor

    
    private val poseHistory = ArrayDeque<Pose2d>(100)

    private val estimatedPoseWriter = DownsampledWriter("ESTIMATED_POSE", 50000000)
    private val targetPoseWriter = DownsampledWriter("TARGET_POSE", 50000000)
    private val driveCommandWriter = DownsampledWriter("DRIVE_COMMAND", 50000000)
    private val mecanumCommandWriter = DownsampledWriter("MECANUM_COMMAND", 50000000)

    init {
        throwIfModulesAreOutdated(hardwareMap)

        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx::class.java, "leftFront")
        leftBack = hardwareMap.get(DcMotorEx::class.java, "leftBack")
        rightBack = hardwareMap.get(DcMotorEx::class.java, "rightBack")
        rightFront = hardwareMap.get(DcMotorEx::class.java, "rightFront")

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        // TODO: reverse motor directions if needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE)
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD)
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD)
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE)

        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        write("MECANUM_PARAMS", PARAMS)
    }
    
    private fun getPose(): Pose2d {
        val cp = localizer.currentPosition
        return Pose2d(cp.x.inch(), cp.y.inch(), cp.heading.radians())
    }

    fun setDrivePowers(powers: PoseVelocity2d) {
        val wheelVels = MecanumKinematics(1.0).inverse<Time?>(
            PoseVelocity2dDual.constant<Time?>(powers, 1)
        )

        var maxPowerMag = 1.0
        for (power in wheelVels.all()) {
            maxPowerMag = max(maxPowerMag, power.value())
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag)
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag)
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag)
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag)
    }

    inner class FollowTrajectoryAction(val timeTrajectory: TimeTrajectory) : Action {
        private var beginTs = -1.0

        private val xPoints: DoubleArray
        private val yPoints: DoubleArray

        init {
            val disps = range(
                0.0, timeTrajectory.path.length(),
                max(2, ceil(timeTrajectory.path.length() / 2).toInt())
            )
            xPoints = DoubleArray(disps.size)
            yPoints = DoubleArray(disps.size)
            for (i in disps.indices) {
                val p = timeTrajectory.path.get(disps[i], 1).value()
                xPoints[i] = p.position.x
                yPoints[i] = p.position.y
            }
        }

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0.0)
                leftBack.setPower(0.0)
                rightBack.setPower(0.0)
                rightFront.setPower(0.0)

                return false
            }

            val txWorldTarget: Pose2dDual<Time> = timeTrajectory.get(t)
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate()

            val command: PoseVelocity2dDual<Time> = HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                .compute(txWorldTarget, getPose(), robotVelRobot)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels = kinematics.inverse<Time>(command)
            val voltage = voltageSensor.getVoltage()

            val feedforward = MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
                )
            )

            leftFront.setPower(leftFrontPower)
            leftBack.setPower(leftBackPower)
            rightBack.setPower(rightBackPower)
            rightFront.setPower(rightFrontPower)

            p.put("x", getPose().position.x)
            p.put("y", getPose().position.y)
            p.put("heading (deg)", Math.toDegrees(getPose().heading.toDouble()))

            val error = txWorldTarget.value().minusExp(getPose())
            p.put("xError", error.position.x)
            p.put("yError", error.position.y)
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()))

            // only draw when active; only one drive action should be active at a time
            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, getPose())

            c.setStroke("#4CAF50FF")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)

            return true
        }

        override fun preview(fieldOverlay: Canvas) {
            fieldOverlay.setStroke("#4CAF507A")
            fieldOverlay.setStrokeWidth(1)
            fieldOverlay.strokePolyline(xPoints, yPoints)
        }
    }

    inner class TurnAction(private val turn: TimeTurn) : Action {
        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }

            if (t >= turn.duration) {
                leftFront.setPower(0.0)
                leftBack.setPower(0.0)
                rightBack.setPower(0.0)
                rightFront.setPower(0.0)

                return false
            }

            val txWorldTarget: Pose2dDual<Time> = turn.get(t)
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate()

            val command: PoseVelocity2dDual<Time> = HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                .compute(txWorldTarget, getPose(), robotVelRobot)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels = kinematics.inverse<Time>(command)
            val voltage = voltageSensor.getVoltage()
            val feedforward = MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick
            )
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
                )
            )

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage)
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage)
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage)
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage)

            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, getPose())

            c.setStroke("#7C4DFFFF")
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)

            return true
        }

        override fun preview(fieldOverlay: Canvas) {
            fieldOverlay.setStroke("#7C4DFF7A")
            fieldOverlay.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)
        }
    }

    fun updatePoseEstimate(): PoseVelocity2d {
        val cv = localizer.currentVelocity
        poseHistory.add(getPose())

        while (poseHistory.size >= 100) {
            poseHistory.removeFirst()
        }

        estimatedPoseWriter.write(PoseMessage(getPose()))

        return PoseVelocity2d(Vector2d(cv.forward.inch(), cv.right.inch()), cv.turn.radians())
    }

    private fun drawPoseHistory(c: Canvas) {
        val xPoints = DoubleArray(poseHistory.size)
        val yPoints = DoubleArray(poseHistory.size)

        var i = 0
        for (t in poseHistory) {
            xPoints[i] = t.position.x
            yPoints[i] = t.position.y

            i++
        }

        c.setStrokeWidth(1)
        c.setStroke("#3F51B5")
        c.strokePolyline(xPoints, yPoints)
    }

    fun movesBuilder(startPosition: Position): MovesBuilder {
        val beginPose =
            Pose2d(startPosition.x.inch(), startPosition.y.inch(), startPosition.heading.radians())

        return MovesBuilder(
            TrajectoryBuilder(
                TrajectoryBuilderParams(
                    1e-6,
                    ProfileParams(
                        0.25, 0.1, 1e-2
                    )
                ),
                beginPose, 0.0,
                defaultVelConstraint, defaultAccelConstraint,
                IdentityPoseMap()
            )
        )
    }

    fun actionBuilder(beginPose: Pose2d): TrajectoryActionBuilder {
        return TrajectoryActionBuilder(
            { turn: TimeTurn -> TurnAction(turn) },
            { t: TimeTrajectory -> FollowTrajectoryAction(t) },
            TrajectoryBuilderParams(
                1e-6,
                ProfileParams(
                    0.25, 0.1, 1e-2
                )
            ),
            beginPose, 0.0,
            defaultTurnConstraints,
            defaultVelConstraint, defaultAccelConstraint
        )
    }

    companion object {
        
        var PARAMS: Params = Params()
    }
}
