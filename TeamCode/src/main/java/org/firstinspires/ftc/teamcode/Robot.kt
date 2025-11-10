package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.internal.RobotOpMode


class Robot(opmode: RobotOpMode) {
    val mtr = opmode.hardware<DcMotor>("mtr")

    init {
        mtr.direction = DcMotorSimple.Direction.REVERSE
    }
}

fun RobotOpMode.auto() {
    robot.mtr.power = 1.0
    telemetry.addData("Elapsed", "$elapsed")
}

fun RobotOpMode.teleop() {
    robot.mtr.power = -0.5
    telemetry.addData("Elapsed", "$runtime")
}
