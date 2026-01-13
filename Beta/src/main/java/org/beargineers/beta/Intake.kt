package org.beargineers.beta

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Hardware
import org.beargineers.platform.config
import org.beargineers.platform.decode.IntakeMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class Intake(robot: BaseRobot): Hardware(robot) {
    private val intake: DcMotor by hardware("intake")
    private val ballDetector: DistanceSensor by hardware("ball")

    val sensorDistanceTreshold by robot.config(10.0)
    val sensorFramesCount by robot.config(3)

    var mode: IntakeMode = IntakeMode.OFF

    var artifacts: Int = 0
    var isHole: Int = 0
    var sensorSee: Boolean = false
    var ballInIntake: Boolean = false

    override fun init() {
        intake.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun loop() {
        telemetry.addData("Intake", mode.name)

        when(mode) {
            IntakeMode.ON -> {
                setMotorPower(intake, 1.0)
            }
            IntakeMode.OFF -> {
                setMotorPower(intake, 0.0)
            }
            IntakeMode.REVERSE -> {
                setMotorPower(intake, -1.0)
            }
        }

        sensorSee = ballDetector.getDistance(DistanceUnit.CM) < sensorDistanceTreshold

        when {
            sensorSee && !ballInIntake -> {
                ballInIntake = true
                artifacts += 1
                isHole = sensorFramesCount
                if (artifacts == 3) {
                    mode = IntakeMode.OFF
                }
            }
            isHole > 0 && !sensorSee -> {
                isHole -= 1
            }
            !sensorSee -> {
                ballInIntake = false
            }
            else -> {
                isHole = sensorFramesCount
            }
        }

        telemetry.addData("Artifacts", artifacts)
    }

    fun onShoot(){
        artifacts = 0
        mode=IntakeMode.ON
    }

    override fun stop() {
        mode = IntakeMode.OFF
    }
}