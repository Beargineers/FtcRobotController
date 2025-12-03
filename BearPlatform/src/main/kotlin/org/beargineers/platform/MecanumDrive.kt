package org.beargineers.platform

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt

@Configurable
object WheelCorrections {
    var LF: Double = -1.0 // Negative value means this motor's encoder reports negative changes when positive power is applied
    var RF: Double = -0.775
    var LB: Double = 1.0
    var RB: Double = 0.925

    var FORWARD_CM_PER_TICK: Double = 0.05752541667
    var STRAFE_CM_PER_TICK: Double = 0.069 // 0.07096614583 // 0.049 //
}

class MecanumDrive(op: BaseRobot) : Hardware(op), Drivetrain {

    // Match these names in  RC configuration
    val lf: DcMotor by hardware("leftFront")
    val rf: DcMotor by hardware("rightFront")
    val lb: DcMotor by hardware("leftBack")
    val rb: DcMotor by hardware("rightBack")
    private val imu: IMU by hardware("imu")


    val localizerByMotorEncoders: RelativeLocalizer by lazy { MecanumEncodersLocalizers() }

    override fun init() {
        val allMotors = listOf(lf, rf, lb, rb)

        lf.direction = DcMotorSimple.Direction.REVERSE
        lb.direction = DcMotorSimple.Direction.REVERSE
        rf.direction = DcMotorSimple.Direction.FORWARD
        rb.direction = DcMotorSimple.Direction.FORWARD

        allMotors.forEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        (localizerByMotorEncoders as MecanumEncodersLocalizers).init()
    }

    override fun loop() {
        super.loop()
        (localizerByMotorEncoders as MecanumEncodersLocalizers).loop()
    }

    override fun drive(forwardPower: Double, rightPower: Double, turnPower: Double, slow: Boolean) {
        val strafe = rightPower * sqrt(2.0) // strafe is less effective than forward movement with same power applied

        val lfP = forwardPower + strafe + turnPower
        val rfP = forwardPower - strafe - turnPower
        val lbP = forwardPower - strafe + turnPower
        val rbP = forwardPower + strafe - turnPower

        val maxMag = listOf(1.0, lfP, rfP, lbP, rbP).maxOf { abs(it) }

        val limit = if (slow) 0.4 else 1.0
        fun normalize(v: Double) = (v / maxMag) * limit

        val lfpn = normalize(lfP * abs(WheelCorrections.LF))
        setMotorPower(lf, lfpn)
        val rfpn = normalize(rfP * abs(WheelCorrections.RF))
        setMotorPower(rf, rfpn)
        val lbpn = normalize(lbP * abs(WheelCorrections.LB))
        setMotorPower(lb, lbpn)
        val rbpn = normalize(rbP * abs(WheelCorrections.RB))
        setMotorPower(rb, rbpn)

        telemetry.addData(
            "Motor power",
            "lf=%.2f, rf=%.2f, lb=%.2f, rb=%.2f",
            lfpn,
            rfpn,
            lbpn,
            rbpn
        )

        telemetry.addData(
            "Encoders", "lf=%d, rf=%d, lb=%d, rb=%d",
            lf.currentPosition,
            rf.currentPosition,
            lb.currentPosition,
            rb.currentPosition
        )
    }

    override fun stop() = drive(0.0, 0.0, 0.0)

    inner class MecanumEncodersLocalizers() : RelativeLocalizer {
        private var lastLf: Int = 0
        private var lastRf: Int = 0
        private var lastLb: Int = 0
        private var lastRb: Int = 0
        private var lastYaw: Double? = null

        private var currentRelativePosition: RelativePosition = RelativePosition.zero()
        private var currentVelocity: RelativePosition = RelativePosition.zero()
        private var previousTime: Long? = null

        override fun getMovementDelta(): RelativePosition {
            return currentRelativePosition
        }

        override fun getVelocity(): RelativePosition {
            return currentVelocity
        }

        fun init() {
            val logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT
            val usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            val orientationOnRobot = RevHubOrientationOnRobot(logoDirection, usbDirection)

            // Initialize the IMU
            imu.initialize(IMU.Parameters(orientationOnRobot))
        }

        fun loop() {
            val now = System.currentTimeMillis()
            val delta = calculateDelta()

            if (previousTime == null) {
                currentRelativePosition = RelativePosition.zero()
                currentVelocity = RelativePosition.zero()
            }
            else {
                val timeDelta = (now - previousTime!!) / 1000.0
                currentRelativePosition = delta
                currentVelocity = RelativePosition(delta.forward / timeDelta, delta.right / timeDelta, delta.turn / timeDelta, delta.distanceUnit, delta.angleUnit)
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

        private fun calculateDelta(): RelativePosition {
            val yaw = getYawFromQuaternionInRadians()
            val deltaYaw = yaw - (lastYaw ?: yaw)

            val lfp = lf.currentPosition
            val lbp = lb.currentPosition
            val rfp = rf.currentPosition
            val rbp = rb.currentPosition

            val deltaLF = (lfp - lastLf) / WheelCorrections.LF
            val deltaLB = (lbp - lastLb) / WheelCorrections.LB
            val deltaRF = (rfp - lastRf) / WheelCorrections.RF
            val deltaRB = (rbp - lastRb) / WheelCorrections.RB

            val forward = WheelCorrections.FORWARD_CM_PER_TICK * (deltaLF + deltaRF + deltaLB + deltaRB) / 4
            val right = WheelCorrections.STRAFE_CM_PER_TICK * (deltaLF - deltaRF + deltaRB - deltaLB) / (4 * sqrt(
                2.0
            ))

            return RelativePosition(
                forward = forward,
                right = right,
                turn = deltaYaw,
                distanceUnit = DistanceUnit.CM,
                angleUnit = AngleUnit.RADIANS
            ).also {
                lastLf = lfp
                lastLb = lbp
                lastRf = rfp
                lastRb = rbp
                lastYaw = yaw
            }
        }
    }
}