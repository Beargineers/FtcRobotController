package org.beargineers.platform

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.sqrt


class MecanumDrive(robot: BaseRobot) : Hardware(robot), Drivetrain {

    // Match these names in  RC configuration
    val lf: DcMotor by hardware("leftFront")
    val rf: DcMotor by hardware("rightFront")
    val lb: DcMotor by hardware("leftBack")
    val rb: DcMotor by hardware("rightBack")
    private val imu: IMU by hardware("imu")

    val localizerByMotorEncoders: RelativeLocalizer by lazy { MecanumEncodersLocalizers() }
    val config = WheelsConfig()

    override fun init() {
        val allMotors = listOf(lf, rf, lb, rb)

        lf.direction = config.lf_direction
        lb.direction = config.lb_direction
        rf.direction = config.rf_direction
        rb.direction = config.rb_direction

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

        val lfpn = normalize(lfP * config.lf_correction)
        setMotorPower(lf, lfpn)
        val rfpn = normalize(rfP * config.rf_correction)
        setMotorPower(rf, rfpn)
        val lbpn = normalize(lbP * config.lb_correction)
        setMotorPower(lb, lbpn)
        val rbpn = normalize(rbP * config.rb_correction)
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
        private var currentPositionEstimate: Position = Position.zero()

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

        fun init() {
            val logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT
            val usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            val orientationOnRobot = RevHubOrientationOnRobot(logoDirection, usbDirection)

            // Initialize the IMU
            imu.initialize(IMU.Parameters(orientationOnRobot))
        }

        fun loop() {
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

            val lfp = lf.currentPosition
            val lbp = lb.currentPosition
            val rfp = rf.currentPosition
            val rbp = rb.currentPosition

            fun DcMotorSimple.Direction.sign() = if (this == DcMotorSimple.Direction.FORWARD) 1 else -1

            val deltaLF = (lfp - lastLf) / config.lf_correction * config.lf_encoder_direction.sign()
            val deltaLB = (lbp - lastLb) / config.lb_correction * config.lb_encoder_direction.sign()
            val deltaRF = (rfp - lastRf) / config.rf_correction * config.rf_encoder_direction.sign()
            val deltaRB = (rbp - lastRb) / config.rb_correction * config.rb_encoder_direction.sign()


            val forward = config.cm_per_tick_forward * (deltaLF + deltaRF + deltaLB + deltaRB) / 4
            val right = config.cm_per_tick_strafe * (deltaLF - deltaRF + deltaRB - deltaLB) / (4 * sqrt(2.0))

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

    inner class WheelsConfig {
        val lf_direction by robot.config(DcMotorSimple.Direction.REVERSE)
        val rf_direction by robot.config(DcMotorSimple.Direction.FORWARD)
        val lb_direction by robot.config( DcMotorSimple.Direction.REVERSE)
        val rb_direction by robot.config(DcMotorSimple.Direction.FORWARD)

        val lf_encoder_direction by robot.config(DcMotorSimple.Direction.FORWARD)
        val rf_encoder_direction by robot.config(DcMotorSimple.Direction.FORWARD)
        val lb_encoder_direction by robot.config(DcMotorSimple.Direction.FORWARD)
        val rb_encoder_direction by robot.config(DcMotorSimple.Direction.FORWARD)

        val lf_correction by robot.config(1.0)
        val rf_correction by robot.config(1.0)
        val lb_correction by robot.config(1.0)
        val rb_correction by robot.config(1.0)

        val cm_per_tick_forward by robot.config(0.0)
        val cm_per_tick_strafe by robot.config(0.0)
    }

}