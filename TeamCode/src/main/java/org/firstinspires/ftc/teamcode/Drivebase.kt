package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.teamcode.internal.Hardware
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

@Configurable
object WheelCorrections {
    var LF: Double = 1.0
    var RF: Double = 0.8
    var LB: Double = 1.0
    var RB: Double = 0.95
}

class Drivebase(op: OpMode) : Hardware(op) {

    // Match these names in  RC configuration
    val lf: DcMotor by hardware("leftFront")
    val rf: DcMotor by hardware("rightFront")
    val lb: DcMotor by hardware("leftBack")
    val rb: DcMotor by hardware("rightBack")
    val imu: IMU by hardware("imu")

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

    /** function to convert the distance the wheel needs to travel to ticks (the wheels can take in) */
    fun cm_to_ticks(cm: Double): Int {
        val TICKS_PER_REV = 537.7    // example; NEEDS THE ACTUAL VALUE
        val GEAR_REDUCTION = 1.0     // >1 if gear reduces speed (motor->wheel)
        val WHEEL_DIAMETER_CM = 9.6  // example; NEEDS THE ACTUAL VALUE

        val revs = cm / (PI * WHEEL_DIAMETER_CM)
        val ticks = revs * TICKS_PER_REV * GEAR_REDUCTION
        return ticks.toInt()
    }

    /** mecanum: y=forward, x=strafe right, turn=clockwise */
    fun drive(y: Double, x: Double, turn: Double, slow: Boolean = false) {
        // TODO For some reason (motor configuration) the strafe and turn come twisted.
        // TODO We're handling the situation programmatically for now

        val _x = turn
        val _turn = x

        val lfP = y + _x + _turn
        val rfP = y - _x - _turn
        val lbP = y - _x + _turn
        val rbP = y + _x - _turn

        val maxMag = listOf(1.0, lfP, rfP, lbP, rbP).maxOf { abs(it) }

        val limit = if (slow) 0.4 else 1.0
        fun normalize(v: Double) = (v / maxMag) * limit

        val lfpn = normalize(lfP * WheelCorrections.LF)
        lf.power = lfpn
        val rfpn = normalize(rfP * WheelCorrections.RF)
        rf.power = rfpn
        val lbpn = normalize(lbP * WheelCorrections.LB)
        lb.power = lbpn
        val rbpn = normalize(rbP * WheelCorrections.RB)
        rb.power = rbpn

        telemetry.addData("Motor power", "lf=%.2f, rf=%.2f, lb=%.2f, rb=%.2f", lfpn, rfpn, lbpn, rbpn)

        telemetry.addData("Encoders", "lf=%d, rf=%d, lb=%d, rb=%d",
            lf.currentPosition,
            rf.currentPosition,
            lb.currentPosition,
            rb.currentPosition
            )
    }

    /**
    function is taken and adapted from https://www.youtube.com/watch?v=gnSW2QpkGXQ with review of chatGPT
    takes in the x and y coordinates in centimeters relative to the robot, and used mechanum wheels to go there at a certain motor power (requires encoders to work)
     */
    fun goto(y: Double, x: Double, power: Double = 0.6) {
        // initializes motors to run a certain distance
        allMotors.forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.mode = DcMotor.RunMode.RUN_TO_POSITION
        }

        // converts the x and y to angle and the distance
        val distance = sqrt(x.pow(2) + y.pow(2))  // pythagorean theorem
        val angle = atan2(y, x)                   // calculates the angle of travel relative to x

        // determine how much each motor needs to turn, PI/4 = 45˚, the angle of the mechanum wheels
        val lfdRatio = cos(angle - PI / 4)
        val rfdRatio = sin(angle - PI / 4)
        val lbdRatio = sin(angle - PI / 4)
        val rbdRatio = cos(angle - PI / 4)

        // normalize so max ratio = 1
        val maxRatio = listOf(lfdRatio, rfdRatio, lbdRatio, rbdRatio).maxOf { abs(it) }
        val lfdNorm = lfdRatio / maxRatio
        val rfdNorm = rfdRatio / maxRatio
        val lbdNorm = lbdRatio / maxRatio
        val rbdNorm = rbdRatio / maxRatio

        // multiply by distance to get actual travel in cm for each wheel
        val lfd = distance * lfdNorm
        val rfd = distance * rfdNorm
        val lbd = distance * lbdNorm
        val rbd = distance * rbdNorm

        // tells the motor how many revs to move
        lf.targetPosition = lf.currentPosition + cm_to_ticks(lfd)
        rf.targetPosition = rf.currentPosition + cm_to_ticks(rfd)
        lb.targetPosition = lb.currentPosition + cm_to_ticks(lbd)
        rb.targetPosition = rb.currentPosition + cm_to_ticks(rbd)


        // sets powers to the motors so that the robot moves uniformly without turning
        lf.power = power * lfdNorm
        rf.power = power * rfdNorm
        lb.power = power * lbdNorm
        rb.power = power * rbdNorm
    }

    fun turn(degrees: Int, power: Double) {
        // set motors to RUN_TO_POSITION mode
        allMotors.forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.mode = DcMotor.RunMode.RUN_TO_POSITION
        }

        val ratio = 10  // needs testing to get the right ration of cm/˚ (NOT THE ACTUAL VALUE)

        val turncm = degrees * ratio

        // multiply by distance to get actual travel in cm for each wheel
        val lfd = turncm.toDouble()
        val rfd = -turncm.toDouble()
        val lbd = turncm.toDouble()
        val rbd = -turncm.toDouble()

        // tells the motor how many revs to move
        lf.targetPosition = lf.currentPosition + cm_to_ticks(lfd)
        rf.targetPosition = rf.currentPosition + cm_to_ticks(rfd)
        lb.targetPosition = lb.currentPosition + cm_to_ticks(lbd)
        rb.targetPosition = rb.currentPosition + cm_to_ticks(rbd)

        // sets powers
        lf.power = power
        rf.power = -power
        lb.power = power
        rb.power = -power
    }

    override fun stop() = drive(0.0, 0.0, 0.0)
}
