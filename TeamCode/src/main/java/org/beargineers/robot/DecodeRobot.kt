package org.beargineers.robot

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.beargineers.platform.Alliance
import org.beargineers.platform.AprilTagWebcam
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.KalmanFilter
import org.beargineers.platform.MecanumDrive
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.WheelsConfig
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection

@Configurable
object WheelCorrections {
    var LF: Double = -1.0 // Negative value means this motor's encoder reports negative changes when positive power is applied
    var RF: Double = -0.775
    var LB: Double = 1.0
    var RB: Double = 0.925

    var FORWARD_CM_PER_TICK: Double = 0.05752541667
    var STRAFE_CM_PER_TICK: Double = 0.069 // 0.07096614583 // 0.049 //

    fun asConfig(): WheelsConfig = WheelsConfig(
        lf_direction = DcMotorSimple.Direction.REVERSE,
        lb_direction = DcMotorSimple.Direction.REVERSE,
        rf_direction = DcMotorSimple.Direction.FORWARD,
        rb_direction = DcMotorSimple.Direction.FORWARD,

        lf_encoder_direction = DcMotorSimple.Direction.REVERSE,
        rf_encoder_direction = DcMotorSimple.Direction.REVERSE,
        lb_encoder_direction = DcMotorSimple.Direction.FORWARD,
        rb_encoder_direction = DcMotorSimple.Direction.FORWARD,

        lf_correction = LF,
        rf_correction = RF,
        lb_correction = LB,
        rb_correction = RB,
        cm_per_tick_forward = FORWARD_CM_PER_TICK,
        cm_per_tick_strafe = STRAFE_CM_PER_TICK
    )
}

class DecodeRobot(opMode: RobotOpMode<DecodeRobot>) : BaseRobot(opMode) {
    override val drive = MecanumDrive(this, WheelCorrections.asConfig())

    override val relativeLocalizer get() = drive.localizerByMotorEncoders
    override val absoluteLocalizer get() = aprilTags

    override fun configureKalmanFilter(): KalmanFilter {
        return KalmanFilter(
            processNoisePosition = KalmanFilterConfig.PROCESS_NOISE_POSITION,
            processNoiseHeading = KalmanFilterConfig.PROCESS_NOISE_HEADING,
            measurementNoisePosition = KalmanFilterConfig.MEASUREMENT_NOISE_POSITION,
            measurementNoiseHeading = KalmanFilterConfig.MEASUREMENT_NOISE_HEADING
        )
    }

    val aprilTags = AprilTagWebcam(this,
        CameraPosition.forward, CameraPosition.right, CameraPosition.up,
        CameraPosition.pitch, CameraPosition.yaw, CameraPosition.roll)
    val shooter = Shooter(this)
    val intake = Intake(this)

    var goalDistanceCM: Double? = null
    var savedGoalDistanceCM: Double = 90.0

    override fun loop() {
        super.loop()

        if (isMoving()) {
            goalDistanceCM = null
        }

        val target = findTarget()
        if (target != null) {
            goalDistanceCM = target.ftcPose.range
        }

        telemetry.addData("Goal distance", goalDistanceCM ?: "goal not found")
    }

    fun findTarget(): AprilTagDetection? {
        return when (opMode.alliance) {
            Alliance.RED -> findRedTarget()
            Alliance.BLUE -> findBlueTarget()
        }
    }

    fun findRedTarget() : AprilTagDetection? {
        return aprilTags.getAprilReadings(24).singleOrNull()
    }

    fun findBlueTarget() : AprilTagDetection? {
        return aprilTags.getAprilReadings(20).singleOrNull()
    }
}