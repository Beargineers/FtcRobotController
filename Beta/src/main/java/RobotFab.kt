import org.beargineers.beta.BetaRobot
import org.beargineers.beta.R
import org.beargineers.platform.RobotFactory
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.decode.DecodeRobot

@Suppress("unused")
class RobotFab : RobotFactory<DecodeRobot> {
    override val configResource: Int = R.raw.config

    override fun createRobot(op: RobotOpMode<DecodeRobot>): DecodeRobot {
        return BetaRobot(op)
    }
}