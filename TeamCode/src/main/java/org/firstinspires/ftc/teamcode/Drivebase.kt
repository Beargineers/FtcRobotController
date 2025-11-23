package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.internal.Hardware
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

@Configurable
object WheelCorrections {
    var LF: Double = 1.0
    var RF: Double = 0.775
    var LB: Double = 1.0
    var RB: Double = 0.925

    var FORWARD_CM_PER_TICK: Double = 0.05752541667
    var STRAFE_CM_PER_TICK: Double =  0.069 // 0.07096614583 // 0.049 //

    var MINIMAL_WHEEL_POWER: Double = 0.12
}

class Drivebase(op: OpMode) : Hardware(op) {

    // Match these names in  RC configuration
    val lf: DcMotor by hardware("leftFront")
    val rf: DcMotor by hardware("rightFront")
    val lb: DcMotor by hardware("leftBack")
    val rb: DcMotor by hardware("rightBack")
    val imu: IMU by hardware("imu")

    var lastLf:Int = 0
    var lastRf:Int = 0
    var lastLb:Int = 0
    var lastRb:Int = 0
    var lastYaw: Double? = null

    val allMotors = listOf(lf, rf, lb, rb)

    init {
        lf.direction = DcMotorSimple.Direction.REVERSE
        lb.direction = DcMotorSimple.Direction.REVERSE
        rf.direction = DcMotorSimple.Direction.FORWARD
        rb.direction = DcMotorSimple.Direction.FORWARD

        allMotors.forEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        val logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT
        val usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        val orientationOnRobot = RevHubOrientationOnRobot(logoDirection, usbDirection)

        // Initialize the IMU
        imu.initialize(IMU.Parameters(orientationOnRobot))
    }

    fun drive(forward: Double, right: Double, turn: Double, slow: Boolean = false) {
        // TODO For some reason (motor configuration) the strafe and turn come twisted.
        // TODO We're handling the situation programmatically for now

        val strafe = turn * sqrt(2.0) // strafe is less effective than forward movement with same power applied
        val cw_turn = right

        val lfP = forward + strafe + cw_turn
        val rfP = forward - strafe - cw_turn
        val lbP = forward - strafe + cw_turn
        val rbP = forward + strafe - cw_turn

        val maxMag = listOf(1.0, lfP, rfP, lbP, rbP).maxOf { abs(it) }

        val limit = if (slow) 0.4 else 1.0
        fun normalize(v: Double) = (v / maxMag) * limit

        val lfpn = normalize(lfP * WheelCorrections.LF)
        setMotorPower(lf, lfpn)
        val rfpn = normalize(rfP * WheelCorrections.RF)
        setMotorPower(rf, rfpn)
        val lbpn = normalize(lbP * WheelCorrections.LB)
        setMotorPower(lb, lbpn)
        val rbpn = normalize(rbP * WheelCorrections.RB)
        setMotorPower(rb, rbpn)

        telemetry.addData("Motor power", "lf=%.2f, rf=%.2f, lb=%.2f, rb=%.2f", lfpn, rfpn, lbpn, rbpn)

        telemetry.addData("Encoders", "lf=%d, rf=%d, lb=%d, rb=%d",
            lf.currentPosition,
            rf.currentPosition,
            lb.currentPosition,
            rb.currentPosition
            )
    }

    override fun stop() = drive(0.0, 0.0, 0.0)

    fun getYawFromQuaternionInRadians(): Double {
        val q = imu.robotOrientationAsQuaternion

        // Calculate yaw from quaternion
        val yawRadians = atan2(
            (2 * (q.w * q.z + q.x * q.y)).toDouble(),
            (1 - 2 * (q.y * q.y + q.z * q.z)).toDouble()
        )

        return yawRadians
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
    fun poseChange(oldHeading: Double): Pose2D {
        val yaw = getYawFromQuaternionInRadians()
        val deltaYaw = yaw - (lastYaw ?: yaw)

        val lf = lf.currentPosition
        val lb = lb.currentPosition
        val rf = rf.currentPosition
        val rb = rb.currentPosition

        // The robot is assembled in a way that encoder counters REDUCE when robot is moving forward. That's why we have negative deltas here
        val deltaLF = -(lf - lastLf) / WheelCorrections.LF
        val deltaLB = -(lb - lastLb) / WheelCorrections.LB
        val deltaRF = -(rf - lastRf) / WheelCorrections.RF
        val deltaRB = -(rb - lastRb) / WheelCorrections.RB

        val forward = WheelCorrections.FORWARD_CM_PER_TICK * (deltaLF + deltaRF + deltaLB + deltaRB) / 4
        val right = WheelCorrections.STRAFE_CM_PER_TICK* (deltaLF - deltaRF + deltaLB - deltaRB) / (4 * sqrt(2.0))

        val N = 10
        val fn = forward / N
        val rn = right / N

        var deltaX = 0.0;
        var deltaY = 0.0;

        var heading = oldHeading

        repeat(N) {
            deltaX += fn * cos(heading) + rn * sin(heading)
            deltaY += fn * sin(heading) - rn * cos(heading)

            heading += deltaYaw / N
        }

        return Pose2D(
            // Forward: all wheels contribute equally
            x = deltaX,
            // Strafe: diagonal wheels oppose (LF and RB forward = strafe right)
            y = deltaY,
            // Get yaw in radians to match the angleUnit specification
            heading = deltaYaw,
            distanceUnit = DistanceUnit.CM,
            angleUnit = AngleUnit.RADIANS
        ).normalizeHeading().also {
            lastLf = lf
            lastLb = lb
            lastRf = rf
            lastRb = rb
            lastYaw = yaw
        }
    }
}
