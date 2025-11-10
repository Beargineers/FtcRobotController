package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.internal.Hardware
import kotlin.math.abs


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

    fun stop() = drive(0.0, 0.0, 0.0)
}
