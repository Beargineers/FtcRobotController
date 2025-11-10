package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.internal.Hardware
import kotlin.math.abs
import kotlin.math.*


class Drivebase(map: HardwareMap) : Hardware(map) {
    // Match these names in  RC configuration
    val lf: DcMotor by hardware ("leftFront")
    val rf: DcMotor by hardware("rightFront")
    val lb: DcMotor by hardware("leftBack")
    val rb: DcMotor by hardware("rightBack")

    init {
        lf.direction = DcMotorSimple.Direction.REVERSE
        lb.direction = DcMotorSimple.Direction.REVERSE
        rf.direction = DcMotorSimple.Direction.FORWARD
        rb.direction = DcMotorSimple.Direction.FORWARD

        listOf(lf, rf, lb, rb).forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
    }

    /** mecanum: y=forward, x=strafe right, turn=clockwise */
    fun drive(y: Double, x: Double, turn: Double, slow: Boolean = false) {
        val lfP = y + x + turn
        val rfP = y - x - turn
        val lbP = y - x + turn
        val rbP = y + x - turn

        val maxMag = listOf(1.0, lfP, rfP, lbP, rbP).maxOf { abs(it)  }

        val limit = if (slow) 0.4 else 1.0
        fun normalize(v: Double) = (v / maxMag) * limit

        lf.power = normalize(lfP)
        rf.power = normalize(rfP)
        lb.power = normalize(lbP)
        rb.power = normalize(rbP)
    }
    // function is taken and adapted from https://www.youtube.com/watch?v=gnSW2QpkGXQ with review of chatGPT
    //takes in the x and y coordinates in centimeters relative to the robot, and used mechanum wheels to go there at a certain motor power (requires encoders to work)
    fun go_to(y: Double, x: Double, power: Double) {
        // initializes motors to run a certain distance
        listOf(lf, rf, lb, rb).forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        // function to convert the distance the wheel needs to travel to ticks (the wheels can take in)
        fun cm_to_ticks(cm: Double): Int {
            val TICKS_PER_REV = 537.7    // example; NEEDS THE ACTUAL VALUE
            val GEAR_REDUCTION = 1.0     // >1 if gear reduces speed (motor->wheel)
            val WHEEL_DIAMETER_CM = 9.6  // example; NEEDS THE ACTUAL VALUE

            val revs = cm / (PI * WHEEL_DIAMETER_CM)
            val ticks = revs * TICKS_PER_REV * GEAR_REDUCTION
            return ticks.toInt()
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
        val maxRatio = max(max(abs(lfdRatio), abs(rfdRatio)), max(abs(lbdRatio), abs(rbdRatio)))
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

        // set motors to RUN_TO_POSITION mode
        listOf(lf, rf, lb, rb).forEach {
            it.mode = DcMotor.RunMode.RUN_TO_POSITION
        }

        // sets powers to the motors so that the robot moves uniformly without turning
        lf.power = power * lfdNorm
        rf.power = power * rfdNorm
        lb.power = power * lbdNorm
        rb.power = power * rbdNorm
    }


    fun stop() = drive(0.0, 0.0, 0.0)
}
