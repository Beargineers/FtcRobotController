package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.internal.RobotOpModeBase


abstract class Robot() : RobotOpModeBase() {
    val mtr : DcMotor by hardware()

    override fun init() {
        mtr.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }
}