package org.beargineers.platform

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt


class MecanumDrive(robot: BaseRobot) : Hardware(robot), Drivetrain {

    // Match these names in  RC configuration
    val lf: DcMotorEx by hardware("leftFront")
    val rf: DcMotorEx by hardware("rightFront")
    val lb: DcMotorEx by hardware("leftBack")
    val rb: DcMotorEx by hardware("rightBack")

    var ticksPerSecond: Int = 0

    override fun init() {
        val allMotors = listOf(lf, rf, lb, rb)

        lf.direction = WheelsConfig.lf_direction
        lb.direction = WheelsConfig.lb_direction
        rf.direction = WheelsConfig.rf_direction
        rb.direction = WheelsConfig.rb_direction

        ticksPerSecond = lf.motorType.achieveableMaxTicksPerSecondRounded


        allMotors.forEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
    }

    override fun drive(forwardPower: Double, rightPower: Double, turnPower: Double) {
        val strafe = rightPower * sqrt(2.0) // strafe is less effective than forward movement with same power applied

        val lfP = forwardPower + strafe + turnPower
        val rfP = forwardPower - strafe - turnPower
        val lbP = forwardPower - strafe + turnPower
        val rbP = forwardPower + strafe - turnPower

        val maxMag = listOf(1.0, lfP, rfP, lbP, rbP).maxOf { abs(it) }

        val limit = robot.targetSpeed * WheelsConfig.topSpeed
        fun normalize(v: Double) = roundMotorPower((v * ticksPerSecond / maxMag) * limit)

        lf.velocity = normalize(lfP * WheelsConfig.lf_correction)
        rf.velocity = normalize(rfP * WheelsConfig.rf_correction)
        lb.velocity = normalize(lbP * WheelsConfig.lb_correction)
        rb.velocity = normalize(rbP * WheelsConfig.rb_correction)
    }

    override fun driveByPowerAndAngle(theta: Double, power: Double, turn: Double) {
        val power = power * robot.targetSpeed * WheelsConfig.topSpeed
        val turn = turn * robot.targetSpeed * WheelsConfig.topSpeed

        val sin = sin(theta + Math.PI / 4)
        val cos = cos(theta + Math.PI / 4)
        val max = max(abs(sin), abs(cos))

        val scale = max(1.0, abs(power) + abs(turn))

        val lfP = (power * cos / max + turn) / scale
        val rfP = (power * sin / max - turn) / scale
        val lrp = (power * sin / max + turn) / scale
        val rbp = (power * cos / max - turn) / scale

        setMotorPowers(lfP, rfP, lrp, rbp)
    }

    private fun setMotorPowers(lfP: Double, rfP: Double, lrp: Double, rbp: Double) {
        lf.power = roundMotorPower(lfP * WheelsConfig.lf_correction)
        rf.power = roundMotorPower(rfP * WheelsConfig.rf_correction)
        lb.power = roundMotorPower(lrp * WheelsConfig.lb_correction)
        rb.power = roundMotorPower(rbp * WheelsConfig.rb_correction)
    }

    override fun stop() = setMotorPowers(0.0, 0.0, 0.0, 0.0)
}

class MecanumEncodersLocalizers(robot: BaseRobot, val wheels: MecanumDrive) : Hardware(robot), RelativeLocalizer {
    private var currentPositionEstimate: Position = Position.zero()
    private val imu: IMU by hardware("imu")


    private var lastLf: Int = 0
    private var lastRf: Int = 0
    private var lastLb: Int = 0
    private var lastRb: Int = 0
    private var lastYaw: Double? = null

    private var currentVelocity: RelativePosition = RelativePosition.zero()
    private var previousTime: Long? = null

    override fun getVelocity(): RelativePosition {
        return currentVelocity
    }

    override fun getPosition(): Position {
        return currentPositionEstimate
    }

    override fun updatePositionEstimate(position: Position) {
        currentPositionEstimate = position
    }

    override fun init() {
        val logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT
        val usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        val orientationOnRobot = RevHubOrientationOnRobot(logoDirection, usbDirection)

        // Initialize the IMU
        imu.initialize(IMU.Parameters(orientationOnRobot))
    }

    override fun loop() {
        val now = System.currentTimeMillis()

        if (previousTime == null) {
            currentVelocity = RelativePosition.zero()
        }
        else {
            val timeDelta = (now - previousTime!!) / 1000.0
            val robotDelta = calculateRobotPositionDelta()
            val positionDelta = positionChange(robotDelta)
            currentPositionEstimate += positionDelta
            currentVelocity = RelativePosition(robotDelta.forward / timeDelta, robotDelta.right / timeDelta, robotDelta.turn / timeDelta)
        }

        previousTime = now
    }

    private fun getYawFromQuaternionInRadians(): Double {
        val q = imu.robotOrientationAsQuaternion

        // Calculate yaw from quaternion
        val yawRadians = atan2(
            (2 * (q.w * q.z + q.x * q.y)).toDouble(),
            (1 - 2 * (q.y * q.y + q.z * q.z)).toDouble()
        )

        return yawRadians
    }

    private fun calculateRobotPositionDelta(): RelativePosition {
        val yaw = getYawFromQuaternionInRadians()
        val deltaYaw = yaw - (lastYaw ?: yaw)

        val lfp = wheels.lf.currentPosition
        val lbp = wheels.lb.currentPosition
        val rfp = wheels.rf.currentPosition
        val rbp = wheels.rb.currentPosition

        fun DcMotorSimple.Direction.sign() = if (this == DcMotorSimple.Direction.FORWARD) 1 else -1

        val deltaLF = (lfp - lastLf) / WheelsConfig.lf_correction * WheelsConfig.lf_encoder_direction.sign()
        val deltaLB = (lbp - lastLb) / WheelsConfig.lb_correction * WheelsConfig.lb_encoder_direction.sign()
        val deltaRF = (rfp - lastRf) / WheelsConfig.rf_correction * WheelsConfig.rf_encoder_direction.sign()
        val deltaRB = (rbp - lastRb) / WheelsConfig.rb_correction * WheelsConfig.rb_encoder_direction.sign()


        val forward = WheelsConfig.cm_per_tick_forward * (deltaLF + deltaRF + deltaLB + deltaRB) / 4
        val right = WheelsConfig.cm_per_tick_strafe * (deltaLF - deltaRF + deltaRB - deltaLB) / (4 * sqrt(2.0))

        return RelativePosition(
            forward = forward.cm,
            right = right.cm,
            turn = deltaYaw.radians
        ).also {
            lastLf = lfp
            lastLb = lbp
            lastRf = rfp
            lastRb = rbp
            lastYaw = yaw
        }
    }

    /**
     * Calculates the change in robot pose based on encoder readings.
     *
     * Uses mecanum wheel forward kinematics to determine the robot's movement
     * in the robot frame since the last encoder reset.
     *
     * Forward kinematics for mecanum drive:
     * - Forward/backward (y): All wheels contribute equally
     * - Strafe left/right (x): Diagonal wheels oppose each other
     * - Heading: From IMU yaw angle
     *
     * @return Pose2D representing the change in position and orientation
     */
    private fun positionChange(move: RelativePosition): Position {
        val (forward, right, deltaYaw) = move

        val N = 10.0
        val fn = forward / N
        val rn = right / N

        var deltaX = 0.cm
        var deltaY = 0.cm

        val oldHeading = currentPositionEstimate.heading
        var heading = oldHeading

        repeat(N.toInt()) {
            deltaX += fn * cos(heading) + rn * sin(heading)
            deltaY += fn * sin(heading) - rn * cos(heading)

            heading += deltaYaw / N
        }

        return Position(
            // Forward: all wheels contribute equally
            x = deltaX,
            // Strafe: diagonal wheels oppose (LF and RB forward = strafe right)
            y = deltaY,
            // Get yaw in radians to match the angleUnit specification
            heading = deltaYaw
        ).normalizeHeading()
    }
}

object WheelsConfig {
    val topSpeed by config(1.0)
    val lf_direction by config(DcMotorSimple.Direction.REVERSE)
    val rf_direction by config(DcMotorSimple.Direction.FORWARD)
    val lb_direction by config( DcMotorSimple.Direction.REVERSE)
    val rb_direction by config(DcMotorSimple.Direction.FORWARD)

    val lf_encoder_direction by config(DcMotorSimple.Direction.FORWARD)
    val rf_encoder_direction by config(DcMotorSimple.Direction.FORWARD)
    val lb_encoder_direction by config(DcMotorSimple.Direction.FORWARD)
    val rb_encoder_direction by config(DcMotorSimple.Direction.FORWARD)

    val lf_correction by config(1.0)
    val rf_correction by config(1.0)
    val lb_correction by config(1.0)
    val rb_correction by config(1.0)

    val cm_per_tick_forward by config(0.0)
    val cm_per_tick_strafe by config(0.0)
}
