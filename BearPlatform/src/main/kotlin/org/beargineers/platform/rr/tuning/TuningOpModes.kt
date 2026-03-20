package org.beargineers.platform.rr.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig
import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.AngularRampLogger
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger
import com.acmerobotics.roadrunner.ftc.DriveType
import com.acmerobotics.roadrunner.ftc.DriveView
import com.acmerobotics.roadrunner.ftc.DriveViewFactory
import com.acmerobotics.roadrunner.ftc.EncoderGroup
import com.acmerobotics.roadrunner.ftc.EncoderRef
import com.acmerobotics.roadrunner.ftc.ForwardPushTest
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger
import com.acmerobotics.roadrunner.ftc.LateralRampLogger
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger
import com.acmerobotics.roadrunner.ftc.PinpointEncoderGroup
import com.acmerobotics.roadrunner.ftc.PinpointIMU
import com.acmerobotics.roadrunner.ftc.PinpointView
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.beargineers.platform.RobotCentricPosition
import org.beargineers.platform.inch
import org.beargineers.platform.radians
import org.beargineers.platform.rr.PinpointLocalizer
import org.beargineers.platform.rr.RRMecanumDrive
import org.beargineers.platform.rr.RRMecanumTuning
import org.beargineers.platform.rr.SimpleLocalizer
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta

object TuningOpModes {
    const val GROUP: String = "quickstart"
    const val DISABLED: Boolean = false

    private fun metaForClass(cls: Class<out OpMode?>): OpModeMeta? {
        return OpModeMeta.Builder()
            .setName(cls.getSimpleName())
            .setGroup(GROUP)
            .setFlavor(OpModeMeta.Flavor.TELEOP)
            .build()
    }

    private fun makePinpointView(pl: PinpointLocalizer): PinpointView {
        return object : PinpointView {
            private var driverParDirection: GoBildaPinpointDriver.EncoderDirection = pl.initialParDirection
            private var driverPerpDirection: GoBildaPinpointDriver.EncoderDirection = pl.initialPerpDirection

            override var parDirection: DcMotorSimple.Direction
                get() = if (driverParDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD) DcMotorSimple.Direction.FORWARD else DcMotorSimple.Direction.REVERSE
                set(value) {
                    driverParDirection =
                        if (value == DcMotorSimple.Direction.FORWARD) GoBildaPinpointDriver.EncoderDirection.FORWARD else GoBildaPinpointDriver.EncoderDirection.REVERSED
                    pl.driver.setEncoderDirections(driverParDirection, driverPerpDirection)
                }

            override var perpDirection: DcMotorSimple.Direction
                get() = if (driverPerpDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD) DcMotorSimple.Direction.FORWARD else DcMotorSimple.Direction.REVERSE
                set(value) {
                    driverPerpDirection =
                        if (value == DcMotorSimple.Direction.FORWARD) GoBildaPinpointDriver.EncoderDirection.FORWARD else GoBildaPinpointDriver.EncoderDirection.REVERSED
                    pl.driver.setEncoderDirections(driverParDirection, driverPerpDirection)
                }

            override fun update() {
                pl.driver.update()
            }

            override fun getParEncoderPosition(): Int {
                return pl.driver.encoderX
            }

            override fun getPerpEncoderPosition(): Int {
                return pl.driver.encoderY
            }

            override fun getHeadingVelocity(unit: UnnormalizedAngleUnit): Float {
                return pl.driver.getHeadingVelocity(unit).toFloat()
            }
        }
    }

    class SimplePinpointLocalizer(val pl: PinpointLocalizer) : SimpleLocalizer {
        override val currentPosition: org.beargineers.platform.Position get()  {
            val pose = pl.getPose()
            return org.beargineers.platform.Position(pose.position.x.inch, pose.position.y.inch, pose.heading.log().radians)
        }

        override val currentVelocity: RobotCentricPosition get() {
            val vel = pl.update()
            return RobotCentricPosition(vel.linearVel.x.inch, vel.linearVel.y.inch, vel.angVel.radians)
        }
    }

    @OpModeRegistrar
    @JvmStatic
    fun register(manager: OpModeManager) {
        if (DISABLED) return

        val dvf: DriveViewFactory = object : DriveViewFactory {
            override fun make(h: HardwareMap): DriveView {
                val pl = PinpointLocalizer(h, Pose2d(0.0, 0.0, 0.0))

                val md = RRMecanumDrive(h, SimplePinpointLocalizer(pl))

                val encoderGroups = mutableListOf<EncoderGroup>()
                val leftEncs = mutableListOf<EncoderRef>()
                val rightEncs = mutableListOf<EncoderRef>()
                val parEncs = mutableListOf<EncoderRef>()
                val perpEncs = mutableListOf<EncoderRef>()
                val pv = makePinpointView(pl)
                encoderGroups.add(PinpointEncoderGroup(pv))
                parEncs.add(EncoderRef(0, 0))
                perpEncs.add(EncoderRef(0, 1))
                val lazyImu = PinpointIMU(pv)

                return DriveView(
                    DriveType.MECANUM,
                    RRMecanumTuning.inPerTick,
                    RRMecanumTuning.maxWheelVel.inch(),
                    RRMecanumTuning.minProfileAccel.inch(),
                    RRMecanumTuning.maxProfileAccel.inch(),
                    encoderGroups,
                    listOf(md.leftFront, md.leftBack),
                    listOf(md.rightFront, md.rightBack),
                    leftEncs,
                    rightEncs,
                    parEncs,
                    perpEncs,
                    lazyImu,
                    md.voltageSensor,
                    {
                        MotorFeedforward(
                            RRMecanumTuning.kS,
                            RRMecanumTuning.kV / RRMecanumTuning.inPerTick,
                            RRMecanumTuning.kA / RRMecanumTuning.inPerTick
                        )
                    },
                    0
                )
            }
        }

        manager.register(metaForClass(AngularRampLogger::class.java), AngularRampLogger(dvf))
        manager.register(metaForClass(ForwardPushTest::class.java), ForwardPushTest(dvf))
        manager.register(metaForClass(ForwardRampLogger::class.java), ForwardRampLogger(dvf))
        manager.register(metaForClass(LateralRampLogger::class.java), LateralRampLogger(dvf))
        manager.register(
            metaForClass(ManualFeedforwardTuner::class.java),
            ManualFeedforwardTuner(dvf)
        )
        manager.register(
            metaForClass(MecanumMotorDirectionDebugger::class.java),
            MecanumMotorDirectionDebugger(dvf)
        )
        manager.register(
            metaForClass(DeadWheelDirectionDebugger::class.java),
            DeadWheelDirectionDebugger(dvf)
        )

        manager.register(
            metaForClass(ManualFeedbackTuner::class.java),
            ManualFeedbackTuner::class.java
        )
        manager.register(metaForClass(SplineTest::class.java), SplineTest::class.java)
        manager.register(metaForClass(LocalizationTest::class.java), LocalizationTest::class.java)

        FtcDashboard.getInstance()
            .withConfigRoot { configRoot: CustomVariable ->
                for (c in listOf(
                    AngularRampLogger::class.java,
                    ForwardRampLogger::class.java,
                    LateralRampLogger::class.java,
                    ManualFeedforwardTuner::class.java,
                    MecanumMotorDirectionDebugger::class.java,
                    ManualFeedbackTuner::class.java
                )) {
                    configRoot.putVariable(
                        c.getSimpleName(),
                        ReflectionConfig.createVariableFromClass(c)
                    )
                }
            }
    }
}
