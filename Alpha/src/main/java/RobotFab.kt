import org.beargineers.platform.RobotFactory
import org.beargineers.platform.RobotOpMode
import org.beargineers.platform.decode.DecodeRobot
import org.beargineers.robot.AlphaRobot

class RobotFab : RobotFactory<DecodeRobot> {
    override fun createRobot(op: RobotOpMode<DecodeRobot>): DecodeRobot {
        return AlphaRobot(op)
    }
}