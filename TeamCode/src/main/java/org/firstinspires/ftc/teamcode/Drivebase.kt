package org.firstinspires.ftc.teamcode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min


class Drivebase(private val opMode: LinearOpMode) {
    // Match these names in  RC configuration
    val lf = opMode.hardwareMap.get(DcMotor::class.java, "leftFront")
    val rf = opMode.hardwareMap.get(DcMotor::class.java, "rightFront")
    val lb = opMode.hardwareMap.get(DcMotor::class.java, "leftBack")
    val rb = opMode.hardwareMap.get(DcMotor::class.java, "rightBack")

    init {

        lf.direction = DcMotorSimple.Direction.REVERSE
        lb.direction = DcMotorSimple.Direction.REVERSE
        rf.direction = DcMotorSimple.Direction.FORWARD
        rb.direction = DcMotorSimple.Direction.FORWARD

        listOf(lf, rf, lb, rb).forEach {
            it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    /** mecanum: y=forward, x=strafe right, turn=clockwise */
    fun drive(y: Double, x: Double, turn: Double, limit: Double = 1.0) {
        var lfP = y + x + turn
        var rfP = y - x - turn
        var lbP = y - x + turn
        var rbP = y + x - turn

        val maxMag = max(1.0, max(max(abs(lfP), abs(rfP)), max(abs(lbP), abs(rbP))))
        lfP = clip(lfP / maxMag, -limit, limit)
        rfP = clip(rfP / maxMag, -limit, limit)
        lbP = clip(lbP / maxMag, -limit, limit)
        rbP = clip(rbP / maxMag, -limit, limit)

        lf.power = lfP; rf.power = rfP; lb.power = lbP; rb.power = rbP
    }

    fun stop() = drive(0.0, 0.0, 0.0)

    private fun clip(v: Double, lo: Double, hi: Double) = min(hi, max(lo, v))
}
