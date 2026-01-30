package org.beargineers.beta

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.LED
import org.beargineers.platform.BaseRobot
import org.beargineers.platform.Hardware
import org.beargineers.platform.config
import org.beargineers.platform.decode.IntakeMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

class Intake(robot: BaseRobot): Hardware(robot) {
    private val intake: DcMotor by hardware("intake")
    private val ballDetector: DistanceSensor by hardware("ball")

    val ballInThreshold by config(13.0)
    val ballOutThreshold by config(17.0)

    val led1green: LED by hardware<LED>("led1green")
    val led1red by hardware<LED>()
    val led2green by hardware<LED>()
    val led2red by hardware<LED>()
    val led3green by hardware<LED>()
    val led3red by hardware<LED>()

    var mode: IntakeMode = IntakeMode.OFF


    val ballCounter = BallCounter(ballInThreshold, ballOutThreshold) {
        artifacts++
        if (artifacts == 3) {
            mode = IntakeMode.OFF
            robot.opMode.gamepad1.rumble(300)
        }
    }

    var artifacts = 0

    override fun init() {
        intake.direction = DcMotorSimple.Direction.REVERSE
        led1red.off()
        led3red.off()
        led2red.off()
    }

    override fun loop() {
        telemetry.addData("Intake", mode.name)

        intake.power = when (mode) {
            IntakeMode.ON -> {
                1.0
            }

            IntakeMode.OFF -> {
                0.0
            }

            IntakeMode.REVERSE -> {
                -1.0
            }
        }

        val distance = ballDetector.getDistance(DistanceUnit.CM)
        ballCounter.update(distance)

        robot.panelsTelemetry.addData("Sensor Distance", distance)
        telemetry.addData("Artifacts", artifacts)

        LedIndicator.ledMods.ledCounter = artifacts
    }
    fun onShoot(){
        artifacts = 0
        mode=IntakeMode.ON
        robot.opMode.gamepad1.rumble(300)
    }

    override fun stop() {
        mode = IntakeMode.OFF
    }


class BallCounter(val ballInThreshold: Double, val ballOutThreshold: Double, val onArtifact: () -> Unit) {
    var ballIsIn = false

    fun update(distance: Double) {
        if (!ballIsIn && distance < ballInThreshold) {
            ballIsIn = true
            onArtifact()
        }
        else if (ballIsIn && distance > ballOutThreshold) {
            ballIsIn = false
        }
    }
}}
